#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MotorOdomMonitor(Node):
    def __init__(self):
        super().__init__("motor_odom_monitor")

        # Последние значения из /m_odom1 ... /m_odom6.
        self.motor_odom = [None] * 6

        # Храним подписки в отдельном списке.
        self.motor_subscriptions = []

        for motor_index in range(6):
            topic_name = f"/m_odom{motor_index + 1}"

            subscription = self.create_subscription(
                Float32,
                topic_name,
                self.make_callback(motor_index),
                10,
            )

            self.motor_subscriptions.append(subscription)

        # Обновляем таблицу 5 раз в секунду.
        self.print_timer = self.create_timer(
            0.1,
            self.print_values,
        )

    def make_callback(self, motor_index):
        def callback(msg):
            self.motor_odom[motor_index] = msg.data

        return callback

    def format_value(self, value):
        """Подготавливает значение для вывода в таблице."""

        if value is None:
            return "---"

        return f"{value:.3f}"

    def print_values(self):
        """Выводит расположение и скорости всех колёс."""

        # Очищаем терминал перед отрисовкой новой таблицы.
        print("\033[2J\033[H", end="")

        print("       Угловые скорости колёс, рад/с")
        print()
        print("┌──────────────────┬──────────────────┐")
        print("│      СЛЕВА       │      СПРАВА      │")
        print("├──────────────────┼──────────────────┤")

        for row in range(3):
            left_motor = row          # Колёса 1, 2, 3
            right_motor = row + 3     # Колёса 4, 5, 6

            left_value = self.format_value(
                self.motor_odom[left_motor]
            )
            right_value = self.format_value(
                self.motor_odom[right_motor]
            )

            print(
                f"│ Колесо {left_motor + 1}: {left_value:>8} "
                f"│ Колесо {right_motor + 1}: {right_value:>8} │"
            )

        print("└──────────────────┴──────────────────┘")
        print()
        print("Для завершения нажмите Ctrl+C", flush=True)


def main(args=None):
    rclpy.init(args=args)

    node = MotorOdomMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nПрограмма остановлена")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()