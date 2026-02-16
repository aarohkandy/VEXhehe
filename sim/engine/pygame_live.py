from __future__ import annotations

import math


class PygameLiveViewer:
    """Live renderer driven directly by simulation state each timestep."""

    def __init__(self, width: int = 1000, height: int = 760):
        self.enabled = False
        self._quit = False
        self.width = width
        self.height = height
        self.path_x: list[float] = []
        self.path_y: list[float] = []

        self.min_x = -1.0
        self.max_x = 1.0
        self.min_y = -1.0
        self.max_y = 1.0

        try:
            import pygame

            self.pygame = pygame
            pygame.init()
            self.screen = pygame.display.set_mode((width, height))
            pygame.display.set_caption("VEX Live Simulation")
            self.clock = pygame.time.Clock()
            self.font = pygame.font.SysFont("monospace", 18)
            self.enabled = True
        except Exception:
            self.enabled = False

    def should_close(self) -> bool:
        return self._quit

    def update(
        self,
        t_s: float,
        x_m: float,
        y_m: float,
        theta_rad: float,
        status_text: str = "",
        target_xy: tuple[float, float] | None = None,
        left_cmd: float = 0.0,
        right_cmd: float = 0.0,
    ) -> None:
        if not self.enabled:
            return

        pg = self.pygame
        for ev in pg.event.get():
            if ev.type == pg.QUIT:
                self._quit = True

        self.path_x.append(x_m)
        self.path_y.append(y_m)

        self.min_x = min(self.min_x, x_m - 0.3)
        self.max_x = max(self.max_x, x_m + 0.3)
        self.min_y = min(self.min_y, y_m - 0.3)
        self.max_y = max(self.max_y, y_m + 0.3)

        bg = (20, 24, 30)
        fg = (228, 233, 240)
        path_c = (74, 164, 255)
        bot_c = (255, 194, 66)
        head_c = (255, 92, 92)

        self.screen.fill(bg)
        self._draw_grid((46, 54, 68))

        if len(self.path_x) > 1:
            pts = [self._to_px(px, py) for px, py in zip(self.path_x, self.path_y)]
            pg.draw.lines(self.screen, path_c, False, pts, 2)

        L = 0.35
        W = 0.30
        corners = []
        for lx, ly in ((+L / 2, +W / 2), (+L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, +W / 2)):
            wx = x_m + lx * math.cos(theta_rad) - ly * math.sin(theta_rad)
            wy = y_m + lx * math.sin(theta_rad) + ly * math.cos(theta_rad)
            corners.append(self._to_px(wx, wy))

        pg.draw.polygon(self.screen, bot_c, corners, width=2)
        hx = x_m + 0.22 * math.cos(theta_rad)
        hy = y_m + 0.22 * math.sin(theta_rad)
        pg.draw.line(self.screen, head_c, self._to_px(x_m, y_m), self._to_px(hx, hy), 3)

        txt = f"LIVE SIM t={t_s:6.2f}s"
        self.screen.blit(self.font.render(txt, True, fg), (14, 12))
        self.screen.blit(self.font.render(f"x={x_m:+.3f}  y={y_m:+.3f}  th={theta_rad:+.3f}", True, fg), (14, 36))
        if status_text:
            self.screen.blit(self.font.render(status_text, True, (140, 235, 170)), (14, 60))
        self._draw_cmd_bar(14, self.height - 72, 180, 14, left_cmd, "L CMD", (110, 220, 255))
        self._draw_cmd_bar(14, self.height - 44, 180, 14, right_cmd, "R CMD", (255, 190, 110))

        if target_xy is not None:
            tx, ty = target_xy
            tpx, tpy = self._to_px(tx, ty)
            self.pygame.draw.circle(self.screen, (255, 95, 215), (tpx, tpy), 6, width=2)
            self.screen.blit(self.font.render("TARGET", True, (255, 95, 215)), (tpx + 8, tpy - 8))

        pg.display.flip()
        self.clock.tick(120)

    def close(self) -> None:
        if not self.enabled:
            return
        self.pygame.quit()

    def _to_px(self, x: float, y: float) -> tuple[int, int]:
        span_x = max(1e-6, self.max_x - self.min_x)
        span_y = max(1e-6, self.max_y - self.min_y)
        nx = (x - self.min_x) / span_x
        ny = (y - self.min_y) / span_y
        px = int(nx * (self.width - 80) + 40)
        py = int((1.0 - ny) * (self.height - 80) + 40)
        return px, py

    def _draw_grid(self, color: tuple[int, int, int]) -> None:
        pg = self.pygame
        n = 10
        for i in range(n + 1):
            x = int(40 + i * ((self.width - 80) / n))
            y = int(40 + i * ((self.height - 80) / n))
            pg.draw.line(self.screen, color, (x, 40), (x, self.height - 40), 1)
            pg.draw.line(self.screen, color, (40, y), (self.width - 40, y), 1)

    def _draw_cmd_bar(
        self,
        x: int,
        y: int,
        w: int,
        h: int,
        cmd: float,
        label: str,
        color: tuple[int, int, int],
    ) -> None:
        pg = self.pygame
        cmd = max(-1.0, min(1.0, cmd))
        center = x + (w // 2)
        pg.draw.rect(self.screen, (70, 75, 88), (x, y, w, h), border_radius=4)
        pg.draw.line(self.screen, (160, 165, 178), (center, y - 2), (center, y + h + 2), 1)
        if cmd >= 0:
            fill_w = int((w // 2) * cmd)
            pg.draw.rect(self.screen, color, (center, y, fill_w, h), border_radius=4)
        else:
            fill_w = int((w // 2) * abs(cmd))
            pg.draw.rect(self.screen, color, (center - fill_w, y, fill_w, h), border_radius=4)
        txt = f"{label}: {cmd:+.2f}"
        self.screen.blit(self.font.render(txt, True, (225, 230, 238)), (x + w + 10, y - 2))
