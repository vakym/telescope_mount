/**
 * @file test_calibration_json.cpp
 * @brief Контракт JSON калибровки: saved и persistence — разные поля.
 */

#include <cstring>

#include "check.hpp"
#include "web/calibration_json.hpp"

namespace {

web::CalibrationView makeView()
{
    web::CalibrationView view{};
    view.state = calibration::SessionState::Review;
    view.axis = motion::AxisId::Azimuth;
    view.direction = motion::Direction::Forward;
    view.session_id = 42;
    view.revision = 7;
    view.motion_generation = 3;
    view.revolutions = 2;
    view.repeats = 2;
    view.measurement_count = 2;
    view.hold_active = false;
    view.result.ok = true;
    view.result.degrees_per_half_step = 0.087890625;
    view.result.half_steps_per_revolution = 4096.0;
    view.result.sample_count = 2;
    view.result.total_revolutions = 4;
    view.result.spread_ppm = 120;
    view.result.max_deviation_ppm = 90;
    view.result.repeatability = calibration::Repeatability::Good;

    view.scale[0].nanodegrees_per_half_step = 87890625;
    view.scale[0].source = calibration::AxisScaleSource::User;
    view.scale[0].persistence = calibration::Persistence::Verified;
    view.scale[0].saved = true;
    view.scale[0].sample_count = 2;
    view.scale[0].total_revolutions = 4;
    view.scale[0].spread_ppm = 120;
    view.nominal_half_steps_per_revolution[0] = 4096U;

    view.scale[1].nanodegrees_per_half_step = 87890625;
    view.scale[1].source = calibration::AxisScaleSource::ConfigNominal;
    view.scale[1].persistence = calibration::Persistence::Absent;
    view.scale[1].saved = false;
    view.nominal_half_steps_per_revolution[1] = 4096U;
    return view;
}

void writesStatusDocument()
{
    const web::CalibrationView view = makeView();
    char buffer[web::kMaxCalibrationJsonBytes];
    std::size_t written = 0;
    CHECK(web::calibrationStatusJson(view, buffer, sizeof(buffer), &written) ==
          web::CalibrationJsonResult::Ok);
    CHECK_GT(written, 0U);
    CHECK(buffer[0] == '{');
    CHECK(buffer[written - 1] == '}');

    CHECK(std::strstr(buffer, "\"state\":\"review\"") != nullptr);
    CHECK(std::strstr(buffer, "\"id\":42") != nullptr);
    CHECK(std::strstr(buffer, "\"source\":\"user\"") != nullptr);
    CHECK(std::strstr(buffer, "\"saved\":true") != nullptr);
    CHECK(std::strstr(buffer, "\"persistence\":\"verified\"") != nullptr);
    // Конфигурационный номинал не выдаётся за сохранённую калибровку.
    CHECK(std::strstr(buffer, "\"source\":\"config_nominal\"") != nullptr);
    CHECK(std::strstr(buffer, "\"saved\":false") != nullptr);
    CHECK(std::strstr(buffer, "\"persistence\":\"absent\"") != nullptr);
    CHECK(std::strstr(buffer, "\"repeatability\":\"good\"") != nullptr);
}

void overflowIsExplicit()
{
    const web::CalibrationView view = makeView();
    char small[64];
    std::size_t written = 123;
    CHECK(web::calibrationStatusJson(view, small, sizeof(small), &written) ==
          web::CalibrationJsonResult::Overflow);
    CHECK(written == 0U);
    CHECK(small[0] == '\0');
}

}  // namespace

int main()
{
    writesStatusDocument();
    overflowIsExplicit();
    return TEST_RESULT();
}
