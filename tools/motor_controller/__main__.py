import math

from textual.app import App, ComposeResult
from textual.containers import Grid
from textual.reactive import reactive
from textual.widgets import Button, Footer, Header, Markdown, Static

from .formatters import format_control_log
from .services import ServicesCollection


class MotorLog(Markdown):
    "Motor log"


class MotorDesc(Static):
    "Motor description"


class TextRect(Static):
    """A widget to display text."""


class MotorView(Static):
    """A motor widget"""

    desc = reactive("Неизвестно", init=False)
    log = reactive("None", init=False)
    speed_diff = reactive(0, init=False)
    state = reactive(None, init=False)
    power = reactive(0, init=False)

    target_speed = 0
    speed = 0

    async def update_view(self):
        log = await self.sc[self.motor_id]['log control']()
        config = await self.sc[self.motor_id]['get config']()
        if log is None:
            return
        self.target_speed = log.log_item.target_speed.value
        self.speed = log.log_item.speed.value
        self.speed_diff = abs(self.speed - self.target_speed)
        self.log = format_control_log(log, config)
        self.state = await self.sc.get_state(self.motor_id)

        V = 24 * (log.log_item.PWM.value / 2000)

        self.power = sum(
            [
                math.cos(math.pi / 3) * abs(V) * abs(I_X.value)
                for I_X in (log.log_item.I_A, log.log_item.I_B, log.log_item.I_C)
            ]
        )

    async def on_mount(self) -> None:
        self.update_timer = self.set_interval(2 / 60, self.update_view)

    def __init__(self, sc, motor_id, desc=""):
        super().__init__()
        self.sc = sc
        self.motor_id = motor_id
        self.desc = desc

    def watch_state(self):
        if self.state is None:
            self.desc = "Неизвестно"
        elif self.state:
            self.desc = "Запущен"
        else:
            self.desc = "Остановлен"

    def watch_power(self):
        prct = self.power / 240 * 100
        color = (
            'red'
            if (prct > 60)
            else ('orange' if (prct > 40) else ('yellow' if (prct > 20) else 'green'))
        )
        self.power_rect.styles.background = f"{color} {prct}%"
        self.power_rect.update(f"Мощность: {self.power:.2f}")

    def watch_desc(self):
        self.main_text.update(f"{self.motor_id}, {self.desc}")

    def watch_log(self):
        self.motor_log.update(self.log)

    def watch_speed_diff(self):
        self.speed_diff_rect.update(f"diff скорости: {self.speed_diff:.3f}")
        if abs(self.speed_diff) > abs(self.target_speed / 10) and self.state:
            self.speed_diff_rect.styles.background = "red 15%"
        else:
            self.speed_diff_rect.styles.background = None

    def compose(self) -> ComposeResult:
        yield Button("Старт", id="mstart", variant="success")
        self.main_text = MotorDesc(f"{self.motor_id}, {self.desc}")
        yield self.main_text
        yield Button("Стоп", id="mstop", variant="error")

        self.speed_diff_rect = TextRect('Отставание скорости')
        yield self.speed_diff_rect

        self.power_rect = TextRect('Мощность')
        yield self.power_rect

        yield TextRect('TODO')

        self.motor_log = MotorLog(self.log)
        yield self.motor_log

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = event.button.id

        if button_id == "mstart":
            await self.sc.set_state(self.motor_id, 1)
        elif button_id == "mstop":
            await self.sc.set_state(self.motor_id, 0)


class MotorControl(App):
    """An app to monitor motors state"""

    CSS_PATH = "app.css"
    BINDINGS = [
        ("R", "reset", "Ресет"),
        ("Q", "quit", "Выход"),
        ("S", "stop_all", "Остановить все"),
    ]

    def __init__(self):
        super().__init__()
        self.dark = True
        self.sc = None

    async def on_mount(self) -> None:
        self.sc = ServicesCollection()
        self.sc.start()

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header()
        yield Footer()

        self.motor_views = [
            MotorView(ServicesCollection(), 2, "Неизвестно"),
            MotorView(ServicesCollection(), 4, "Неизвестно"),
            MotorView(ServicesCollection(), 8, "Неизвестно"),
            MotorView(ServicesCollection(), 16, "Неизвестно"),
        ]

        grid = Grid(*self.motor_views)
        grid.styles.grid_size_rows = 2
        grid.styles.grid_size_columns = 2

        yield grid

    def action_quit(self):
        self.exit()

    async def action_stop_all(self):
        await self.sc.set_state_all(0)

    def action_reset(self):
        pass


if __name__ == "__main__":
    app = MotorControl()
    app.run()
