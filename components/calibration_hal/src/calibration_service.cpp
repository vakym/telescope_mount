/**
 * @file calibration_service.cpp
 * @brief Сервис калибровки: задача, сессия, удержание и сохранение.
 */

#include "calibration_hal/calibration_service.hpp"

#include "esp_random.h"

#include <cmath>
#include <cstring>

#include "calibration/math.hpp"
#include "calibration/record.hpp"
#include "esp_log.h"
#include "sdkconfig.h"

namespace calibration_hal {
namespace {

constexpr const char *kTag = "calibration";
constexpr std::uint32_t kTaskStackBytes = 5120;
constexpr UBaseType_t kTaskPriority = 5;
constexpr calibration::Micros kStopConfirmTimeoutUs = 3'000'000;
constexpr calibration::Micros kReleaseConfirmUs = 300'000;
/// Минимальный срок ненулевой уставки: меньше — считаем, что разрешение
/// закончилось, и обрабатываем это как истечение.
constexpr calibration::Micros kMinMoveLeaseUs = 50'000;
constexpr calibration::Micros kFlashWindowTimeoutUs = 5'000'000;
constexpr calibration::Micros kReadinessLossMarginUs = 1'500'000;
constexpr calibration::Micros kClaimRetryUs = 250'000;

std::size_t axisIndex(motion::AxisId axis) noexcept
{
    return static_cast<std::size_t>(axis);
}

}  // namespace

CalibrationService::CalibrationService(control_hal::ControlService &control,
                                       state_hal::StateHub &hub)
    : control_(control), port_(control, hub)
{
    mutex_ = xSemaphoreCreateMutexStatic(&mutex_storage_);
    params_ = makeParams();
    for (std::size_t i = 0; i < 2; ++i) {
        refreshScale(static_cast<motion::AxisId>(i));
    }
}

CalibrationService::~CalibrationService()
{
    running_.store(false);
    if (task_ != nullptr) {
        vTaskDelay(pdMS_TO_TICKS(50));
        task_ = nullptr;
    }
}

calibration::CalibrationParams CalibrationService::makeParams() const
{
    calibration::CalibrationParams params{};
    params.nominal_half_steps_per_revolution[axisIndex(motion::AxisId::Azimuth)] =
        CONFIG_CAL_AZ_NOMINAL_HALF_STEPS_PER_REVOLUTION;
    params.nominal_half_steps_per_revolution[axisIndex(motion::AxisId::Altitude)] =
        CONFIG_CAL_ALT_NOMINAL_HALF_STEPS_PER_REVOLUTION;
    params.min_measurements = CONFIG_CAL_MIN_MEASUREMENTS;
    params.max_measurements = CONFIG_CAL_MAX_MEASUREMENTS;
    params.max_spread_ppm = CONFIG_CAL_MAX_SPREAD_PPM;
    return params;
}

bool CalibrationService::lock(TickType_t wait) const
{
    return mutex_ != nullptr && xSemaphoreTake(mutex_, wait) == pdTRUE;
}

void CalibrationService::unlock() const
{
    (void)xSemaphoreGive(mutex_);
}

bool CalibrationService::tokenValid(std::uint64_t token) const noexcept
{
    return session_id_ != 0U && token != 0U && token == session_token_;
}

bool CalibrationService::finishSessionLocked(calibration::Micros now)
{
    if (released_) {
        return true;
    }
    move_active_ = false;
    stop_pending_ = false;
    released_ = releaseSession(now);
    return released_;
}

esp_err_t CalibrationService::start()
{
    if (running_.exchange(true)) {
        return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t error = store_.init();
    if (error != ESP_OK) {
        ESP_LOGE(kTag, "NVS init failed: %s; calibration unavailable",
                 esp_err_to_name(error));
    }
    for (std::size_t i = 0; i < 2; ++i) {
        refreshScale(static_cast<motion::AxisId>(i));
    }
    const BaseType_t created = xTaskCreatePinnedToCore(
        &CalibrationService::taskEntry, "calibration", kTaskStackBytes, this,
        kTaskPriority, &task_, tskNO_AFFINITY);
    if (created != pdPASS) {
        running_.store(false);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void CalibrationService::taskEntry(void *arg)
{
    static_cast<CalibrationService *>(arg)->run();
}

void CalibrationService::run()
{
    while (running_.load()) {
        step();
        vTaskDelay(pdMS_TO_TICKS(CONFIG_CAL_TASK_PERIOD_MS));
    }
    task_ = nullptr;
    vTaskDelete(nullptr);
}

void CalibrationService::refreshScale(motion::AxisId axis)
{
    const std::size_t index = axisIndex(axis);
    AxisScaleStatus status{};
    status.nominal_half_steps_per_revolution =
        params_.nominal_half_steps_per_revolution[index];
    const NvsStore::Loaded loaded = store_.load(axis);
    if (loaded.present) {
        const calibration::Record &record = loaded.record;
        status.scale.nanodegrees_per_half_step = record.nanodegrees_per_half_step;
        status.scale.saved = record.userCalibrated();
        status.scale.source = record.userCalibrated()
                                  ? calibration::AxisScaleSource::User
                                  : calibration::AxisScaleSource::SavedNominal;
        status.scale.persistence = calibration::Persistence::Verified;
        status.scale.sample_count = record.sample_count;
        status.scale.total_revolutions = record.total_revolutions;
        status.scale.spread_ppm = record.spread_ppm;
    } else {
        status.scale.nanodegrees_per_half_step = calibration::toNanodegrees(
            calibration::nominalDegreesPerHalfStep(
                status.nominal_half_steps_per_revolution));
        status.scale.source = calibration::AxisScaleSource::ConfigNominal;
        status.scale.saved = false;
        status.scale.persistence = loaded.error == ESP_OK
                                       ? calibration::Persistence::Absent
                                       : calibration::Persistence::Failed;
    }
    scales_[index] = status;
}

AxisScaleStatus CalibrationService::axisScale(motion::AxisId axis) const
{
    AxisScaleStatus status{};
    if (!lock()) {
        return status;
    }
    status = scales_[axisIndex(axis)];
    unlock();
    return status;
}

CalibrationStatus CalibrationService::status() const
{
    CalibrationStatus out{};
    if (!lock()) {
        return out;
    }
    out.state = session_.state();
    out.error = session_.error();
    out.invalid_reason = session_.lastInvalidReason();
    out.session_id = session_id_;
    out.revision = session_.revision();
    out.motion_generation = control_.motionGeneration();
    out.axis = session_.config().axis;
    out.direction = session_.config().direction;
    out.revolutions = session_.config().revolutions;
    out.repeats = session_.config().repeats;
    out.measurement_count = static_cast<std::uint32_t>(session_.measurementCount());
    const calibration::Measurement *measurements = session_.measurements();
    for (std::size_t i = 0; i < session_.measurementCount() && i < calibration::kMaxMeasurements;
         ++i) {
        out.measurements[i] = measurements[i];
    }
    out.result = session_.result();
    out.mark_recheck_required = session_.markRecheckRequired();
    out.hold_active = hold_started_;
    out.input_ready = control_.calibrationInputReady();
    out.flash_window_active = control_.flashWindowActive();
    const calibration::Micros now = port_.nowUs();
    const calibration::Micros until =
        readiness_refresh_us_ + CONFIG_CAL_READINESS_LEASE_MS * 1000ULL;
    out.lease_remaining_ms = (out.input_ready && until > now)
                                 ? static_cast<std::uint32_t>((until - now) / 1000ULL)
                                 : 0;
    out.axes[0] = scales_[0];
    out.axes[1] = scales_[1];
    unlock();
    return out;
}

CommandOutcome CalibrationService::createSession(const SessionRequest &request,
                                                 std::uint32_t &session_id,
                                                 std::uint64_t &token)
{
    session_id = 0;
    token = 0;
    calibration::SessionConfig config{};
    config.axis = request.axis;
    config.direction = request.direction;
    config.revolutions = request.revolutions;
    config.repeats = request.repeats;
    if (!calibration::validConfig(config, params_)) {
        return CommandOutcome::InvalidArgument;
    }
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session_.active() && !session_.terminal()) {
        unlock();
        return CommandOutcome::Busy;
    }
    if (session_.terminal()) {
        // Новая сессия не начинается поверх незавершённого освобождения.
        if (!finishSessionLocked(port_.nowUs())) {
            unlock();
            return CommandOutcome::Busy;
        }
        session_.close();
    }
    if (!session_.start(config, params_)) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    session_id_ = next_session_id_++;
    if (next_session_id_ == 0U) {
        next_session_id_ = 1;
    }
    // Токен не связан с номером сессии и не выводится из публичных данных.
    do {
        session_token_ = (static_cast<std::uint64_t>(esp_random()) << 32U) |
                         static_cast<std::uint64_t>(esp_random());
    } while (session_token_ == 0U);
    token = session_token_;
    session_id = session_id_;
    ownership_claimed_ = false;
    claim_retry_us_ = 0;
    hold_started_ = false;
    reference_epoch_ = 0;
    readiness_refresh_us_ = 0;
    hold_deadline_us_ = 0;
    move_active_ = false;
    stop_pending_ = false;
    stop_request_id_ = 0;
    released_ = false;
    last_state_ = calibration::SessionState::Preparing;
    state_since_us_ = port_.nowUs();
    last_activity_us_ = state_since_us_;
    last_ready_us_ = 0;
    unlock();
    return CommandOutcome::Ok;
}

CommandOutcome CalibrationService::closeSession(std::uint32_t session, std::uint64_t token)
{
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session != session_id_ || session_id_ == 0U) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!tokenValid(token)) {
        unlock();
        return CommandOutcome::Forbidden;
    }
    if (!session_.terminal()) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    if (!finishSessionLocked(port_.nowUs())) {
        // Освобождение не подтверждено: сессия остаётся открытой для повтора.
        unlock();
        return CommandOutcome::Busy;
    }
    session_.close();
    session_id_ = 0;
    session_token_ = 0;
    unlock();
    return CommandOutcome::Ok;
}

CommandOutcome CalibrationService::move(std::uint32_t session, std::uint64_t token,
                                        motion::Direction direction, float rate,
                                        std::uint32_t expected_generation)
{
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session != session_id_ || session_id_ == 0U) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!tokenValid(token)) {
        unlock();
        return CommandOutcome::Forbidden;
    }
    // Клиент передаёт модуль скорости; знак задаётся только направлением.
    // Отрицательная уставка отклоняется, чтобы не было двух источников знака.
    if (!std::isfinite(rate) || rate <= 0.0F ||
        rate > static_cast<float>(CONFIG_CAL_MAX_RATE_HALF_STEPS_PER_S)) {
        unlock();
        return CommandOutcome::InvalidArgument;
    }
    if (expected_generation != control_.motionGeneration()) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!session_.noteMove(direction, session_.motionGeneration())) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    if (session_.state() == calibration::SessionState::Error) {
        // Реверс в измерении аннулирует сессию, а не запускает движение.
        unlock();
        return CommandOutcome::InvalidState;
    }
    if (!move_active_) {
        // Новое непрерывное удержание кнопки: свой таймер, не от confirmStart.
        continuous_move_started_us_ = port_.nowUs();
    }
    move_active_ = true;
    move_rate_ = rate;
    move_direction_ = direction;
    const calibration::Micros now = port_.nowUs();
    move_until_us_ = now + CONFIG_CAL_MOVE_LEASE_MS * 1000ULL;
    last_activity_us_ = now;
    unlock();
    return CommandOutcome::Ok;
}

bool CalibrationService::requestStopInternal(calibration::Micros now)
{
    next_stop_request_ += 1;
    if (next_stop_request_ == 0U) {
        next_stop_request_ = 1;
    }
    stop_request_id_ = next_stop_request_;
    stop_requested_us_ = now;
    stop_pending_ = true;
    move_active_ = false;
    continuous_move_started_us_ = 0;
    // Штатный останов не должен терять владение из-за остатка короткого
    // lease движения: немедленно обновляем слот нулевой уставкой с полным
    // сроком пригодности источника.
    if (ownership_claimed_ && session_id_ != 0U) {
        (void)port_.setRate(session_id_, session_.config().axis, 0.0F,
                            static_cast<calibration::Micros>(CONFIG_CAL_READINESS_LEASE_MS) *
                                1000ULL);
    }
    return port_.requestStop(stop_request_id_);
}

CommandOutcome CalibrationService::stop(std::uint32_t session, std::uint64_t token)
{
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session != session_id_ || session_id_ == 0U) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!tokenValid(token)) {
        unlock();
        return CommandOutcome::Forbidden;
    }
    if (stop_pending_) {
        unlock();
        return CommandOutcome::Ok;  // идемпотентно
    }
    if (!session_.requestStop()) {
        // Уже остановлено: повтор возвращает текущее состояние.
        unlock();
        return CommandOutcome::Ok;
    }
    const calibration::Micros now = port_.nowUs();
    last_activity_us_ = now;
    (void)requestStopInternal(now);
    unlock();
    return CommandOutcome::Ok;
}

CommandOutcome CalibrationService::confirmStart(std::uint32_t session, std::uint64_t token)
{
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session != session_id_ || session_id_ == 0U) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!tokenValid(token)) {
        unlock();
        return CommandOutcome::Forbidden;
    }
    // Повторная проверка условий в момент пользовательского подтверждения:
    // доверять только раннему снимку нельзя.
    if (!liveReferenceValid(port_.snapshot())) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    if (!session_.confirmStart()) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    const calibration::Micros now = port_.nowUs();
    measure_started_us_ = now;
    last_activity_us_ = now;
    move_active_ = false;
    unlock();
    return CommandOutcome::Ok;
}

CommandOutcome CalibrationService::confirmEnd(std::uint32_t session, std::uint64_t token)
{
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session != session_id_ || session_id_ == 0U) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!tokenValid(token)) {
        unlock();
        return CommandOutcome::Forbidden;
    }
    if (!liveReferenceValid(port_.snapshot())) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    if (!session_.confirmEnd()) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    last_activity_us_ = port_.nowUs();
    unlock();
    return CommandOutcome::Ok;
}

CommandOutcome CalibrationService::resume(std::uint32_t session, std::uint64_t token)
{
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session != session_id_ || session_id_ == 0U) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!tokenValid(token)) {
        unlock();
        return CommandOutcome::Forbidden;
    }
    if (!session_.resume()) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    last_activity_us_ = port_.nowUs();
    unlock();
    return CommandOutcome::Ok;
}

CommandOutcome CalibrationService::abandon(std::uint32_t session, std::uint64_t token)
{
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session != session_id_ || session_id_ == 0U) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!tokenValid(token)) {
        unlock();
        return CommandOutcome::Forbidden;
    }
    if (!session_.abandon()) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    move_active_ = false;
    last_activity_us_ = port_.nowUs();
    unlock();
    return CommandOutcome::Ok;
}

CommandOutcome CalibrationService::cancelSession(std::uint32_t session, std::uint64_t token)
{
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session != session_id_ || session_id_ == 0U) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!tokenValid(token)) {
        unlock();
        return CommandOutcome::Forbidden;
    }
    move_active_ = false;
    if (!session_.cancel()) {
        // Терминальное состояние: отменять нечего.
        unlock();
        return CommandOutcome::InvalidState;
    }
    const calibration::Micros now = port_.nowUs();
    last_activity_us_ = now;
    if (session_.state() == calibration::SessionState::Cancelling && !stop_pending_) {
        (void)requestStopInternal(now);
    }
    unlock();
    return CommandOutcome::Ok;
}

CommandOutcome CalibrationService::deleteMeasurement(std::uint32_t session,
                                                     std::uint64_t token, std::size_t index)
{
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session != session_id_ || session_id_ == 0U) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!tokenValid(token)) {
        unlock();
        return CommandOutcome::Forbidden;
    }
    if (!session_.deleteMeasurement(index)) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    last_activity_us_ = port_.nowUs();
    unlock();
    return CommandOutcome::Ok;
}

CommandOutcome CalibrationService::addRepeat(std::uint32_t session, std::uint64_t token)
{
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session != session_id_ || session_id_ == 0U) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!tokenValid(token)) {
        unlock();
        return CommandOutcome::Forbidden;
    }
    if (!session_.addRepeat()) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    ownership_claimed_ = false;
    hold_started_ = false;
    claim_retry_us_ = 0;
    reference_epoch_ = 0;
    // Повтор требует заново получить владение и удержание: освобождение в
    // Review уже произошло, поэтому фиксация «освобождено» снимается.
    released_ = false;
    last_activity_us_ = port_.nowUs();
    unlock();
    return CommandOutcome::Ok;
}

int CalibrationService::persist(const calibration::CombinedResult &result)
{
    if (!port_.beginFlashWindow(kFlashWindowTimeoutUs)) {
        return 0;
    }
    const motion::AxisId axis = session_.config().axis;
    calibration::Record record =
        calibration::makeCalibratedRecord(result, store_.nextSequence());
    const esp_err_t error = store_.save(axis, record);
    bool verified = false;
    if (error == ESP_OK) {
        const NvsStore::Loaded loaded = store_.load(axis);
        verified = loaded.present && loaded.record.sequence == record.sequence &&
                   loaded.record.userCalibrated() &&
                   loaded.record.nanodegrees_per_half_step ==
                       record.nanodegrees_per_half_step;
    }
    port_.endFlashWindow();
    if (error != ESP_OK) {
        ESP_LOGE(kTag, "save failed: %s", esp_err_to_name(error));
        return 0;
    }
    return verified ? 2 : 1;
}

CommandOutcome CalibrationService::apply(std::uint32_t session, std::uint64_t token,
                                         bool acknowledge_low_repeatability)
{
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session != session_id_ || session_id_ == 0U) {
        unlock();
        return CommandOutcome::StaleSession;
    }
    if (!tokenValid(token)) {
        unlock();
        return CommandOutcome::Forbidden;
    }
    const calibration::ApplyResult applied =
        session_.apply(acknowledge_low_repeatability);
    if (applied == calibration::ApplyResult::RepeatabilityRejected) {
        unlock();
        return CommandOutcome::InvalidArgument;
    }
    if (applied != calibration::ApplyResult::Ok) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    session_.beginSaving();
    const calibration::CombinedResult result = session_.result();
    const int persist_result = persist(result);
    if (persist_result == 2) {
        session_.saveVerified(result);
        refreshScale(session_.config().axis);
    } else if (persist_result == 1) {
        session_.saveUncertain();
    } else {
        session_.saveFailed();
    }
    last_activity_us_ = port_.nowUs();
    unlock();
    return CommandOutcome::Ok;
}

bool CalibrationService::persistNominal(motion::AxisId axis)
{
    const std::uint32_t nominal =
        params_.nominal_half_steps_per_revolution[axisIndex(axis)];
    calibration::Record record = calibration::makeNominalRecord(nominal, store_.nextSequence());
    const esp_err_t error = store_.save(axis, record);
    if (error != ESP_OK) {
        return false;
    }
    const NvsStore::Loaded loaded = store_.load(axis);
    return loaded.present && loaded.record.sequence == record.sequence &&
           loaded.record.nanodegrees_per_half_step == record.nanodegrees_per_half_step;
}

CommandOutcome CalibrationService::resetAxis(motion::AxisId axis, bool confirmed)
{
    if (axisIndex(axis) >= 2) {
        return CommandOutcome::InvalidArgument;
    }
    if (!confirmed) {
        // Необратимый сброс требует явного подтверждения, а не только кнопки.
        return CommandOutcome::InvalidArgument;
    }
    if (!lock()) {
        return CommandOutcome::Busy;
    }
    if (session_.active() && !session_.terminal()) {
        unlock();
        return CommandOutcome::Busy;
    }
    if (!port_.beginFlashWindow(kFlashWindowTimeoutUs)) {
        unlock();
        return CommandOutcome::InvalidState;
    }
    const bool ok = persistNominal(axis);
    port_.endFlashWindow();
    refreshScale(axis);
    unlock();
    return ok ? CommandOutcome::Ok : CommandOutcome::InvalidState;
}

void CalibrationService::refreshReadiness(calibration::Micros now)
{
    if (session_id_ == 0U || session_.terminal()) {
        return;
    }
    if (now - readiness_refresh_us_ <
        static_cast<calibration::Micros>(CONFIG_CAL_READINESS_REFRESH_MS) * 1000ULL) {
        return;
    }
    const bool ready = ownership_claimed_ && hold_started_;
    calibration::Micros lease =
        static_cast<calibration::Micros>(CONFIG_CAL_READINESS_LEASE_MS) * 1000ULL;
    port_.setInputReady(ready, session_id_, now + lease);
    // Арбитражный слот живёт по lease уставки: без периодического продления
    // владение истекло бы через секунду даже без движения. Повторяем текущую
    // уставку, но её срок не выходит за разрешение браузера.
    const float rate = permittedOutputRate(now);
    if (rate != 0.0F) {
        const calibration::Micros remaining = move_until_us_ - now;
        if (remaining < lease) {
            lease = remaining;
        }
    }
    (void)port_.setRate(session_id_, session_.config().axis, rate, lease);
    readiness_refresh_us_ = now;
}

bool CalibrationService::liveReferenceValid(const calibration::Snapshot &snapshot) const noexcept
{
    const std::size_t index = axisIndex(session_.config().axis);
    return !session_.terminal() && ownership_claimed_ && hold_started_ &&
           snapshot.owner == state::SourceId::Calibration && snapshot.input_ready &&
           snapshot.position_trusted[index] && !snapshot.has_fault[index] &&
           snapshot.energized[index];
}

void CalibrationService::validateMeasurement(calibration::Micros now,
                                             const calibration::Snapshot &snapshot)
{
    (void)now;
    if (stop_pending_) {
        // Останов сам меняет уставки и может кратковременно снимать питание:
        // достоверность проверяется после подтверждения.
        return;
    }
    const calibration::SessionState state = session_.state();
    if (state != calibration::SessionState::StartReady &&
        state != calibration::SessionState::Measuring &&
        state != calibration::SessionState::EndReady &&
        state != calibration::SessionState::AwaitingStartStop &&
        state != calibration::SessionState::AwaitingEndStop) {
        return;
    }
    const std::size_t index = axisIndex(session_.config().axis);
    if (snapshot.has_fault[index] || !snapshot.position_trusted[index]) {
        move_active_ = false;
        failSession(calibration::SessionError::MeasurementInvalid);
        return;
    }
    if (reference_epoch_ != 0U && snapshot.command_epoch[index] != reference_epoch_) {
        move_active_ = false;
        failSession(calibration::SessionError::MeasurementInvalid);
        return;
    }
    if (hold_started_ && !snapshot.energized[index]) {
        // Удержание потеряно: точки недостоверны. Пытаемся восстановить
        // удержание; если не удаётся — сессия завершается ошибкой.
        move_active_ = false;
        session_.holdLost();
        hold_started_ = false;
        hold_deadline_us_ = now + static_cast<calibration::Micros>(CONFIG_CAL_MAX_REFERENCE_HOLD_MS) *
                                      1000ULL;
        if (!port_.beginHold(session_.config().axis, session_id_, hold_deadline_us_)) {
            failSession(calibration::SessionError::HoldUnavailable);
            return;
        }
        hold_started_ = true;
    }
}

void CalibrationService::revokeBrowserMotion(calibration::Micros now)
{
    if (!move_active_) {
        return;
    }
    // Браузер перестал продлевать разрешение или оно отозвано досрочно:
    // текущее измерение аннулируется, ось останавливается.
    move_active_ = false;
    continuous_move_started_us_ = 0;
    const calibration::SessionState state = session_.state();
    if (state == calibration::SessionState::Measuring) {
        (void)session_.abandon();
    }
    if (state == calibration::SessionState::Measuring ||
        state == calibration::SessionState::InitialPositioning ||
        state == calibration::SessionState::EndReady) {
        if (session_.requestStop()) {
            (void)requestStopInternal(now);
        } else if (ownership_claimed_ && session_id_ != 0U) {
            // Владение сохраняется: обновляем слот нулевой уставкой.
            (void)port_.setRate(session_id_, session_.config().axis, 0.0F,
                                static_cast<calibration::Micros>(CONFIG_CAL_READINESS_LEASE_MS) *
                                    1000ULL);
        }
    }
}

bool CalibrationService::handleMoveLease(calibration::Micros now)
{
    // Разрешение действует, только пока остаток не меньше минимального
    // срока уставки: иначе короткий lease мог бы уничтожить владение.
    if (!move_active_ || now + kMinMoveLeaseUs <= move_until_us_) {
        return false;
    }
    revokeBrowserMotion(now);
    return true;
}

float CalibrationService::permittedOutputRate(calibration::Micros now) const noexcept
{
    if (!move_active_ || move_until_us_ <= now + kMinMoveLeaseUs) {
        return 0.0F;
    }
    return calibration::signedRate(move_direction_, move_rate_);
}

void CalibrationService::stepActive(calibration::Micros now,
                                    const calibration::Snapshot &snapshot)
{
    if (session_.state() == calibration::SessionState::InitialPositioning) {
        reference_epoch_ = 0;
    }
    // Дедлайны и отзыв разрешения обрабатываются ДО любой публикации
    // уставки: refreshReadiness не должен продлить ненулевое движение.
    (void)handleMoveLease(now);
    if (ownership_claimed_ && snapshot.owner != state::SourceId::Calibration) {
        // Владение отозвано извне (аварийный останов, потеря сессии):
        // сессия не продолжается, удержание будет восстановлено при очистке.
        failSession(calibration::SessionError::HoldUnavailable);
        return;
    }
    if (ownership_claimed_ && !snapshot.permit) {
        // Разрешение движения снято (disarm или истечение владельца).
        move_active_ = false;
        session_.leaseExpired();
        return;
    }
    refreshReadiness(now);
    validateMeasurement(now, snapshot);
    if (session_.terminal()) {
        return;
    }
    driveMove(now, snapshot);
    superviseStop(now, snapshot);
}

void CalibrationService::stepPreparing(calibration::Micros now,
                                       const calibration::Snapshot &snapshot)
{
    // Владение могло быть отозвано аварийным остановом или запоздавшим
    // release предыдущей сессии: тогда claim повторяется немедленно.
    if (ownership_claimed_ && snapshot.owner != state::SourceId::Calibration) {
        ownership_claimed_ = false;
        hold_started_ = false;
        claim_retry_us_ = 0;
    }
    if (!ownership_claimed_) {
        if (snapshot.owner == state::SourceId::Calibration) {
            ownership_claimed_ = true;
        } else if (now >= claim_retry_us_) {
            (void)port_.claim(session_id_);
            claim_retry_us_ = now + kClaimRetryUs;
            return;
        }
    }
    if (!hold_started_) {
        hold_deadline_us_ =
            now + static_cast<calibration::Micros>(CONFIG_CAL_MAX_REFERENCE_HOLD_MS) * 1000ULL;
        if (!port_.beginHold(session_.config().axis, session_id_, hold_deadline_us_)) {
            failSession(calibration::SessionError::HoldUnavailable);
            return;
        }
        hold_started_ = true;
    }
    refreshReadiness(now);
    if (snapshot.owner == state::SourceId::Calibration && snapshot.input_ready) {
        (void)session_.ownerReady();
        last_ready_us_ = now;
        last_activity_us_ = now;
    }
}

void CalibrationService::driveMove(calibration::Micros now,
                                   const calibration::Snapshot &snapshot)
{
    if (!move_active_ || now > move_until_us_) {
        return;  // дедлайн уже обработан в handleMoveLease()
    }
    const calibration::SessionState state = session_.state();
    if (state != calibration::SessionState::InitialPositioning &&
        state != calibration::SessionState::StartReady &&
        state != calibration::SessionState::Measuring &&
        state != calibration::SessionState::EndReady) {
        move_active_ = false;
        return;
    }
    if (!session_.noteMove(move_direction_, session_.motionGeneration())) {
        move_active_ = false;
        return;
    }
    if (state == calibration::SessionState::StartReady) {
        // Движение с подтверждённой стартовой точки аннулирует attestation.
        move_active_ = false;
        continuous_move_started_us_ = 0;
        return;
    }
    // Знак задаётся направлением, модуль — клиентом. Срок уставки не
    // продлевается за дедлайн разрешения браузера.
    const float rate = permittedOutputRate(now);
    if (rate == 0.0F) {
        move_active_ = false;
        return;
    }
    const calibration::Micros remaining = move_until_us_ - now;
    if (remaining < kMinMoveLeaseUs) {
        // Слишком короткий остаток не публикуем: разрешение отозвано.
        revokeBrowserMotion(now);
        return;
    }
    calibration::Micros lease_us =
        static_cast<calibration::Micros>(CONFIG_CAL_MOVE_LEASE_MS) * 1000ULL;
    if (remaining < lease_us) {
        lease_us = remaining;
    }
    (void)port_.setRate(session_id_, session_.config().axis, rate, lease_us);
    if (state == calibration::SessionState::Measuring &&
        continuous_move_started_us_ != 0 &&
        now - continuous_move_started_us_ >
            static_cast<calibration::Micros>(CONFIG_CAL_MAX_CONTINUOUS_MOVE_MS) * 1000ULL) {
        // Предел одного непрерывного движения: принудительный останов.
        if (session_.requestStop()) {
            (void)requestStopInternal(now);
        }
    }
    (void)snapshot;
}

void CalibrationService::superviseStop(calibration::Micros now,
                                       const calibration::Snapshot &snapshot)
{
    if (!stop_pending_) {
        return;
    }
    const calibration::StopAttestation attestation = port_.lastStopAttestation();
    // Подтверждается конкретный запрос, а не подходящий по времени снимок.
    if (attestation.valid && attestation.confirmed &&
        attestation.request_id == stop_request_id_ &&
        attestation.confirmed_us >= stop_requested_us_) {
        stop_pending_ = false;
        last_activity_us_ = now;
        if (session_.state() == calibration::SessionState::Cancelling) {
            session_.cancelStopConfirmed();
            return;
        }
        const std::size_t index = axisIndex(session_.config().axis);
        calibration::ReferenceAttestation reference{};
        reference.valid = attestation.valid;
        reference.position = attestation.position_step[index];
        reference.ticket = attestation.ticket_id[index];
        reference.epoch = attestation.epoch[index];
        reference.position_trusted = attestation.position_trusted[index];
        reference.energized = attestation.energized[index];
        reference.fault = snapshot.has_fault[index];
        const bool reference_ready = hold_started_ && reference.energized;
        const bool accepted = session_.confirmStop(reference, snapshot.command_epoch[index],
                                                   reference_ready);
        if (accepted && session_.state() == calibration::SessionState::StartReady) {
            // Эпоха стартовой точки фиксируется для контроля всего измерения.
            reference_epoch_ = reference.epoch;
        }
        return;
    }
    if (now - stop_requested_us_ > kStopConfirmTimeoutUs) {
        stop_pending_ = false;
        failSession(calibration::SessionError::StopUnconfirmed);
    }
}

void CalibrationService::superviseTimeouts(calibration::Micros now,
                                           const calibration::Snapshot &snapshot)
{
    if (session_.terminal()) {
        return;
    }
    if (session_.state() == calibration::SessionState::StartReady ||
        session_.state() == calibration::SessionState::EndReady) {
        if (now - state_since_us_ >
            static_cast<calibration::Micros>(CONFIG_CAL_REFERENCE_CONFIRM_TIMEOUT_MS) * 1000ULL) {
            failSession(calibration::SessionError::ReferenceConfirmTimeout);
            return;
        }
    }
    if (snapshot.input_ready) {
        last_ready_us_ = now;
    } else if (last_ready_us_ != 0 &&
               session_.state() != calibration::SessionState::Preparing &&
               now - last_ready_us_ > kReadinessLossMarginUs) {
        // Управляющий слой отозвал разрешение (истечение lease, потеря
        // удержания, абсолютный предел): сессия не продолжается.
        failSession(calibration::SessionError::HoldUnavailable);
        return;
    }
    if (session_.state() != calibration::SessionState::Review &&
        now - last_activity_us_ >
            static_cast<calibration::Micros>(CONFIG_CAL_SESSION_IDLE_TIMEOUT_MS) * 1000ULL) {
        move_active_ = false;
        if (session_.cancel() && session_.state() == calibration::SessionState::Cancelling) {
            (void)requestStopInternal(now);
        }
    }
}

void CalibrationService::failSession(calibration::SessionError error)
{
    move_active_ = false;
    stop_pending_ = false;
    session_.fail(error);
}

bool CalibrationService::releaseSession(calibration::Micros now)
{
    port_.setInputReady(false, session_id_, 0);
    readiness_refresh_us_ = 0;
    if (hold_started_) {
        (void)port_.endHold(session_.config().axis, session_id_);
    }
    if (ownership_claimed_) {
        port_.release(session_id_);
    }
    // Ждём подтверждения управляющего слоя, а не только отправки заявок.
    const calibration::Micros deadline = now + kReleaseConfirmUs;
    for (;;) {
        const bool hold_clear =
            !hold_started_ || port_.holdCleared(session_.config().axis);
        const bool owner_clear = !ownership_claimed_ ||
                                 port_.snapshot().owner != state::SourceId::Calibration;
        if (hold_clear && owner_clear) {
            hold_started_ = false;
            ownership_claimed_ = false;
            return true;
        }
        if (port_.nowUs() >= deadline) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

void CalibrationService::step()
{
    if (!lock(10)) {
        return;
    }
    if (!session_.active() && !session_.terminal()) {
        // Inactive без терминального состояния: освобождать нечего, сессии нет.
        unlock();
        return;
    }
    const calibration::Micros now = port_.nowUs();
    const calibration::Snapshot snapshot = port_.snapshot();

    const calibration::SessionState state = session_.state();
    if (state != last_state_) {
        last_state_ = state;
        state_since_us_ = now;
    }

    // Терминальная очистка выполняется и для Cancelled, который не считается
    // active(): ресурсы освобождаются до того, как клиент закроет сессию.
    if (session_.terminal()) {
        finishSessionLocked(now);
        unlock();
        return;
    }

    switch (state) {
        case calibration::SessionState::Preparing:
            stepPreparing(now, snapshot);
            break;
        case calibration::SessionState::Review:
        case calibration::SessionState::Applying:
        case calibration::SessionState::Saving:
            // Удержание и владение снимаются: применение не требует движения.
            if (!released_) {
                move_active_ = false;
                released_ = releaseSession(now);
            }
            break;
        case calibration::SessionState::Cancelling:
            if (!stop_pending_) {
                (void)requestStopInternal(now);
            }
            superviseStop(now, snapshot);
            break;
        default:
            stepActive(now, snapshot);
            break;
    }

    if (!session_.terminal()) {
        superviseTimeouts(now, snapshot);
    }
    unlock();
}

}  // namespace calibration_hal
