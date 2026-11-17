/**
 * @file test_calibration_command.cpp
 * @brief Разбор тела команд калибровки: обязательные поля и диапазоны.
 */

#include <cstring>

#include "check.hpp"
#include "web/calibration_command.hpp"

namespace {

using web::CalibrationCommand;
using web::CalibrationCommandKind;
using web::CalibrationCommandResult;

void parsesCreateSession()
{
    const char *body = "{\"axis\":\"altitude\",\"direction\":\"backward\","
                       "\"revolutions\":3,\"repeats\":2}";
    CalibrationCommand command{};
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::CreateSession, body,
                                       std::strlen(body), command) ==
          CalibrationCommandResult::Ok);
    CHECK(command.axis == 1);
    CHECK(command.direction == 1);
    CHECK(command.revolutions == 3U);
    CHECK(command.repeats == 2U);
}

void parsesMove()
{
    const char *body = "{\"session_id\":5,\"token\":77,\"motion_generation\":9,"
                       "\"direction\":\"forward\",\"rate\":120.5}";
    CalibrationCommand command{};
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::Move, body,
                                       std::strlen(body), command) ==
          CalibrationCommandResult::Ok);
    CHECK(command.session_id == 5U);
    CHECK(command.token == 77U);
    CHECK(command.motion_generation == 9U);
    CHECK(command.direction == 0);
    CHECK_NEAR(command.rate, 120.5, 1e-3);
}

void rejectsMissingField()
{
    const char *body = "{\"session_id\":5}";
    CalibrationCommand command{};
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::Move, body,
                                       std::strlen(body), command) ==
          CalibrationCommandResult::MissingField);
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::Stop, "{}", 2,
                                       command) ==
          CalibrationCommandResult::MissingField);
    // Без токена изменяющая команда не принимается.
    const char *no_token = "{\"session_id\":5}";
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::Stop, no_token,
                                       std::strlen(no_token), command) ==
          CalibrationCommandResult::MissingField);
    // Удаление измерения требует номер.
    const char *no_index = "{\"session_id\":5,\"token\":7}";
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::DeleteMeasurement,
                                       no_index, std::strlen(no_index), command) ==
          CalibrationCommandResult::MissingField);
}

void resetRequiresConfirmation()
{
    CalibrationCommand command{};
    const char *no_confirm = "{\"axis\":\"azimuth\"}";
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::Reset, no_confirm,
                                       std::strlen(no_confirm), command) ==
          CalibrationCommandResult::InvalidValue);
    const char *confirmed = "{\"axis\":\"azimuth\",\"confirm\":true}";
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::Reset, confirmed,
                                       std::strlen(confirmed), command) ==
          CalibrationCommandResult::Ok);
    CHECK(command.confirm);
}

void rejectsInvalidValues()
{
    CalibrationCommand command{};
    CHECK(web::parseCalibrationCommand(
              CalibrationCommandKind::CreateSession,
              "{\"axis\":\"sideways\",\"direction\":\"forward\",\"revolutions\":1,"
              "\"repeats\":2}",
              std::strlen("{\"axis\":\"sideways\",\"direction\":\"forward\",\"revolutions\":1,"
                          "\"repeats\":2}"),
              command) == CalibrationCommandResult::InvalidValue);
    const char *bad_revolutions =
        "{\"axis\":0,\"direction\":0,\"revolutions\":0,\"repeats\":2}";
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::CreateSession,
                                       bad_revolutions, std::strlen(bad_revolutions),
                                       command) ==
          CalibrationCommandResult::InvalidValue);
    const char *zero_rate =
        "{\"session_id\":1,\"token\":9,\"motion_generation\":0,\"direction\":0,\"rate\":0}";
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::Move, zero_rate,
                                       std::strlen(zero_rate), command) ==
          CalibrationCommandResult::InvalidValue);
}

void rejectsMalformedJson()
{
    CalibrationCommand command{};
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::Stop, "not json", 8,
                                       command) ==
          CalibrationCommandResult::InvalidJson);
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::Stop,
                                       "{\"session_id\":1}trailing", 22, command) ==
          CalibrationCommandResult::InvalidJson);
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::Stop, nullptr, 0,
                                       command) ==
          CalibrationCommandResult::InvalidJson);
}

void parsesApplyAcknowledgement()
{
    const char *body = "{\"session_id\":3,\"token\":9,"
                       "\"acknowledge_low_repeatability\":true}";
    CalibrationCommand command{};
    CHECK(web::parseCalibrationCommand(CalibrationCommandKind::Apply, body,
                                       std::strlen(body), command) ==
          CalibrationCommandResult::Ok);
    CHECK(command.acknowledge_low_repeatability);
}

}  // namespace

int main()
{
    parsesCreateSession();
    parsesMove();
    rejectsMissingField();
    resetRequiresConfirmation();
    rejectsInvalidValues();
    rejectsMalformedJson();
    parsesApplyAcknowledgement();
    return TEST_RESULT();
}
