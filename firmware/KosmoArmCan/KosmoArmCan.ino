#include <Servo.h>
#include <VBCoreG4_arduino_system.h>
#include <string.h>

constexpr uint16_t COMMAND_ID = 0x520;      // CAN-идентификатор команд от Raspberry Pi.
constexpr uint8_t PROTOCOL_VERSION = 0xA2;  // Первый байт данных обозначает версию формата.
constexpr uint32_t WATCHDOG_MS = 150;
constexpr uint32_t UPDATE_MS = 5;

// Порядок приводов в CAN-кадре: база, суставы 1–3, вращение схвата, пальцы.
constexpr uint32_t SERVO_PINS[] = {PB14, PB1, PB0, PA6, PA4, PA0};
constexpr size_t JOINT_COUNT = sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]);
constexpr int MIN_ANGLE = 5;
constexpr int HOME_ANGLE = 90;
constexpr int MAX_ANGLE = 175;
constexpr int SPEED_DEG_PER_SEC = 30;
constexpr float MAX_STEP = SPEED_DEG_PER_SEC * UPDATE_MS / 1000.0f;

enum class Command : uint8_t { Hold = 0, Move = 1, Home = 2 };

CanFD can;
FDCAN_HandleTypeDef* fdcan;
Servo servos[JOINT_COUNT];
float angles[JOINT_COUNT];
int8_t rates[JOINT_COUNT];
bool holdSeen = false;
bool initialized = false;
Command mode = Command::Hold;
uint32_t lastCommandMs = 0;
uint32_t lastUpdateMs = 0;

void setup() {
  SystemClock_Config();
  pinMode(LED2, OUTPUT);
  can.init();
  can.write_default_params();
  can.apply_config();
  fdcan = can.get_hfdcan();
  can.default_start();
}

void acceptCommand(const uint8_t* data, uint32_t now) {
  if (data[0] != PROTOCOL_VERSION) {
    return;
  }

  const Command command = static_cast<Command>(data[1]);
  // За байтом команды идут шесть скоростей от −100 до +100 % в порядке SERVO_PINS.
  const uint8_t* rateBytes = data + 2;
  switch (command) {
    case Command::Hold:
      holdSeen = true;
      break;
    case Command::Move:
      if (mode == Command::Hold) {
        // Для начала движения ожидаем сначала Hold, затем Move с нулевыми скоростями.
        constexpr uint8_t zeroRates[JOINT_COUNT] = {};
        if (!holdSeen || memcmp(rateBytes, zeroRates, sizeof(zeroRates)) != 0) return;
      }
      memcpy(rates, rateBytes, sizeof(rates));
      break;
    case Command::Home:
      if (!holdSeen) return;
      break;
    default:
      return;
  }

  if (!initialized && command != Command::Hold) {
    // При первом разрешении управления включаем приводы в стартовой позе.
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
      angles[i] = HOME_ANGLE;
      servos[i].attach(SERVO_PINS[i], HOME_ANGLE);
    }
    initialized = true;
  }
  mode = command;
  lastCommandMs = now;
}

void loop() {
  const uint32_t now = millis();
  if (now - lastCommandMs >= WATCHDOG_MS) {
    mode = Command::Hold;
    holdSeen = false;
  }

  // Читаем до восьми кадров за проход, чтобы вовремя проверять таймаут связи.
  for (int i = 0; i < 8 && HAL_FDCAN_GetRxFifoFillLevel(fdcan, FDCAN_RX_FIFO0); ++i) {
    FDCAN_RxHeaderTypeDef header{};
    uint8_t data[64];
    if (HAL_FDCAN_GetRxMessage(fdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
      break;
    }
    if (header.IdType == FDCAN_STANDARD_ID && header.Identifier == COMMAND_ID &&
        header.RxFrameType == FDCAN_DATA_FRAME && header.FDFormat == FDCAN_CLASSIC_CAN &&
        header.DataLength == FDCAN_DLC_BYTES_8) {
      acceptCommand(data, now);
    }
  }
  digitalWrite(LED2, mode == Command::Hold ? LOW : HIGH);

  if (now - lastUpdateMs < UPDATE_MS) {
    return;
  }
  lastUpdateMs = now;
  if (mode == Command::Hold) {
    return;  // Библиотека Servo продолжает подавать сигнал для удержания последней позы.
  }

  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    // По CH6 приближаемся к стартовому углу, при управлении стиком шаг зависит от скорости.
    const float step = mode == Command::Home
        ? constrain(HOME_ANGLE - angles[i], -MAX_STEP, MAX_STEP)
        : rates[i] * MAX_STEP / 100.0f;
    if (step == 0) continue;
    angles[i] = constrain(angles[i] + step, MIN_ANGLE, MAX_ANGLE);
    servos[i].write(static_cast<int>(angles[i] + 0.5f));
  }
}
