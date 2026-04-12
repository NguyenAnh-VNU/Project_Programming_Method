import pygame
import sys
import math
from collections import deque

# ─── Constants ────────────────────────────────────────────────────────────────
WIDTH, HEIGHT  = 1380, 780
SIM_W          = 900
PANEL_W        = WIDTH - SIM_W
FPS            = 60
PLATE_SIZE     = 150
BALL_RADIUS    = 12
GRAVITY        = 9.81 * 300          # px/s²
MAX_TILT       = 22                  # degrees
FRICTION       = 0.9995              # per-frame rolling friction
MAX_BALL_SPEED = 800.0               # [OPT] velocity cap – prevents runaway

# ─── Bounce constants ─────────────────────────────────────────────────────────
RESTITUTION      = 0.62
BOUNCE_FRICTION  = 0.91
MIN_BOUNCE_SPEED = 30.0
BOUNCE_SPIN      = 0.04

# ─── Orbit mode constants ─────────────────────────────────────────────────────
ORBIT_RADIUS = 160.0
ORBIT_SPEED  = 0.6

# ─── Colors ───────────────────────────────────────────────────────────────────
BG         = (8,  10,  18)
PANEL_BG   = (12, 14,  26)
PLATE_EDGE = (0,  220, 255)
GRID_COL   = (15, 90,  120)
BALL_COL   = (255, 75,  75)
BALL_SHINE = (255, 200, 200)
TEXT_COL   = (180, 210, 255)
ACCENT     = (0,  220, 255)
DANGER     = (255, 70,  70)
SUCCESS    = (50,  255, 140)
SLIDER_BG  = (25,  30,  50)
SLIDER_FG  = (0,  190, 255)
GRAPH_BG   = (12,  16,  32)
BOUNCE_COL = (255, 220, 80)
PUSH_COL   = (120, 255, 160)
ORBIT_COL  = (200, 120, 255)

# ─── Tripod Leg constants ─────────────────────────────────────────────────────
LEG_COUNT     = 3
LEG_ATTACH_R  = PLATE_SIZE
LEG_FOOT_R    = PLATE_SIZE * 1.45
LEG_BASE_Z    = -240
LEG_COL_OUTER = (0,  180, 220)
LEG_COL_INNER = (0,  255, 200)
LEG_JOINT_COL = (255, 220, 60)
LEG_FOOT_COL  = (60, 120, 180)

# [OPT] Precomputed constant — avoids sqrt every frame in boundary check
_PLATE_SIZE_SQ = PLATE_SIZE * PLATE_SIZE


# ─── Camera ───────────────────────────────────────────────────────────────────
class Camera:
    def __init__(self):
        self.azimuth   = 45.0
        self.elevation = 30.0
        self.zoom      = 0.80
        self.pan_x     = 0.0
        self.pan_y     = 0.0
        self._drag     = False
        self._lx = self._ly = 0

    def handle(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
            mx, my = event.pos
            if mx < SIM_W:
                self._drag = True
                self._lx, self._ly = mx, my
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 3:
            self._drag = False
        elif event.type == pygame.MOUSEMOTION and self._drag:
            mx, my = event.pos
            dx, dy = mx - self._lx, my - self._ly
            if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                self.pan_x += dx; self.pan_y += dy
            else:
                self.azimuth   -= dx * 0.4
                self.elevation  = max(10, min(89, self.elevation - dy * 0.3))
            self._lx, self._ly = mx, my
        elif event.type == pygame.MOUSEWHEEL:
            mx, my = pygame.mouse.get_pos()
            if mx < SIM_W:
                self.zoom = max(0.3, min(2.5, self.zoom + event.y * 0.06))

    def project(self, x, y, z, cx, cy):
        az = math.radians(self.azimuth)
        el = math.radians(self.elevation)
        rx =  x * math.cos(az) + y * math.sin(az)
        ry = -x * math.sin(az) + y * math.cos(az)
        sx =  rx
        sy =  ry * math.cos(el) + z * math.sin(el)
        return (int(cx + self.pan_x + sx * self.zoom),
                int(cy + self.pan_y - sy * self.zoom))

    def reset(self):
        self.azimuth = 45.0; self.elevation = 30.0
        self.zoom    = 0.80; self.pan_x = 0.0; self.pan_y = 0.0


# ─── PID Controller ───────────────────────────────────────────────────────────
class PIDController:
    """
    [FIX] Derivative is now EMA-filtered (alpha=0.25) to eliminate the
    large 'derivative kick' that occurred every time pid.reset() was called
    (prev_error jumped from 0 to current_error in one frame).
    [OPT] Integral clamped tighter (+-400 instead of +-600) to reduce windup.
    """
    def __init__(self, kp=0.08, ki=0.0005, kd=0.35):
        self.kp = kp; self.ki = ki; self.kd = kd
        self._integral       = 0.0
        self._prev_error     = 0.0
        self._deriv_filtered = 0.0

    def update(self, error, dt):
        self._integral += error * dt
        self._integral  = max(-400.0, min(400.0, self._integral))
        raw_deriv = (error - self._prev_error) / max(dt, 1e-6)
        self._deriv_filtered = 0.75 * self._deriv_filtered + 0.25 * raw_deriv
        self._prev_error = error
        out = (self.kp * error
               + self.ki * self._integral
               + self.kd * self._deriv_filtered)
        return max(-MAX_TILT, min(MAX_TILT, out))

    def reset(self):
        self._integral = 0.0; self._prev_error = 0.0; self._deriv_filtered = 0.0


# ─── Ball ─────────────────────────────────────────────────────────────────────
class Ball:
    def __init__(self, x=0.0, y=0.0, z=240.0, vx=0.0, vy=0.0, vz=0.0):
        self.x  = float(x);  self.y  = float(y);  self.z  = float(z)
        self.vx = float(vx); self.vy = float(vy); self.vz = float(vz)
        self.on_plate     = False
        self.alive        = True
        self.bounces      = 0
        self.bounce_flash = 0
        self.trail        = deque(maxlen=40)

    def update(self, tilt_x, tilt_y, dt, plate=None, restitution=RESTITUTION):
        if not self.alive:
            return
        if self.bounce_flash > 0:
            self.bounce_flash -= dt * 1000
        self.trail.append((self.x, self.y, self.z))

        # [OPT] Precompute trig once — used in both branches + boundary check
        tx_r   = math.radians(tilt_x)
        ty_r   = math.radians(tilt_y)
        sin_tx = math.sin(tx_r)
        sin_ty = math.sin(ty_r)

        if not self.on_plate:
            # ── Airborne ─────────────────────────────────────────────────────
            # [FIX] Removed incorrect 0.15-factor horizontal gravity when in
            # air. Plate tilt cannot push a ball that is not touching it —
            # only gravity (vz) acts during flight. vx/vy remain constant
            # (parabolic arc). Old code: ax = sin(tilt)*GRAVITY*0.15 was wrong.
            self.vz -= GRAVITY * dt
            self.z  += self.vz * dt
            self.x  += self.vx * dt
            self.y  += self.vy * dt
            plate_z  = self.x * sin_ty - self.y * sin_tx
            if self.z <= plate_z:
                self._bounce(sin_tx, sin_ty, plate_z, plate, restitution)
            if self.z < -800:
                self.alive = False
        else:
            # ── On plate ─────────────────────────────────────────────────────
            ax = sin_ty * GRAVITY
            ay = sin_tx * GRAVITY
            self.vx = (self.vx + ax * dt) * FRICTION
            self.vy = (self.vy + ay * dt) * FRICTION
            # [OPT] Velocity cap — prevents runaway after hard pushes
            spd = math.hypot(self.vx, self.vy)
            if spd > MAX_BALL_SPEED:
                s = MAX_BALL_SPEED / spd
                self.vx *= s; self.vy *= s
            self.x += self.vx * dt
            self.y += self.vy * dt
            self.z  = self.x * sin_ty - self.y * sin_tx
            if self.vz > MIN_BOUNCE_SPEED * 0.1:
                self.on_plate = False
            # [OPT] Squared distance — avoids sqrt every frame
            if self.x * self.x + self.y * self.y > _PLATE_SIZE_SQ:
                self.alive = False

    def _bounce(self, sin_tx, sin_ty, plate_z, plate, restitution):
        # [FIX] Accepts pre-computed sin values + restitution as argument.
        # Removes dependency on global RESTITUTION (was mutated via 'global').
        self.z = plate_z
        impact_speed = abs(self.vz)
        if impact_speed < MIN_BOUNCE_SPEED:
            self.vz = 0.0; self.on_plate = True
        else:
            self.vz  = impact_speed * restitution
            self.vx *= BOUNCE_FRICTION
            self.vy *= BOUNCE_FRICTION
            self.vz += abs(self.vx) * BOUNCE_SPIN
            self.vz += abs(self.vy) * BOUNCE_SPIN
            self.on_plate = False
            self.bounces += 1
            self.bounce_flash = 120
            if plate is not None:
                strength = min(abs(self.vz) / 400, 3.0)
                plate.impact_x -= (self.vy / 300) * strength
                plate.impact_y -= (self.vx / 300) * strength


# ─── Plate ────────────────────────────────────────────────────────────────────
class Plate:
    def __init__(self):
        self.tilt_x    = 0.0; self.tilt_y    = 0.0
        self.impact_x  = 0.0; self.impact_y  = 0.0
        self.ang_vel_x = 0.0; self.ang_vel_y = 0.0

    def update(self, target_tx, target_ty, dt):
        spd = 130 * dt
        self.tilt_x += max(-spd, min(spd, target_tx - self.tilt_x))
        self.tilt_y += max(-spd, min(spd, target_ty - self.tilt_y))
        self.ang_vel_x += self.impact_x;  self.ang_vel_y += self.impact_y
        self.impact_x   = 0.0;            self.impact_y   = 0.0
        self.tilt_x   += self.ang_vel_x * dt * 60
        self.tilt_y   += self.ang_vel_y * dt * 60
        self.ang_vel_x -= self.tilt_x * 0.012
        self.ang_vel_y -= self.tilt_y * 0.012
        self.ang_vel_x *= 0.92; self.ang_vel_y *= 0.92
        self.tilt_x = max(-MAX_TILT, min(MAX_TILT, self.tilt_x))
        self.tilt_y = max(-MAX_TILT, min(MAX_TILT, self.tilt_y))


# ─── UIWidget (base) ──────────────────────────────────────────────────────────
class UIWidget:
    def __init__(self, x, y, w, h, label):
        self.rect = pygame.Rect(x, y, w, h); self.label = label
    def draw(self, surf, font): raise NotImplementedError
    def handle(self, event): pass


# ─── Slider ───────────────────────────────────────────────────────────────────
class Slider(UIWidget):
    def __init__(self, x, y, w, label, vmin, vmax, value, color=None):
        super().__init__(x, y, w, 6, label)
        self.vmin = vmin; self.vmax = vmax; self.value = value
        self.dragging = False; self.color = color or SLIDER_FG

    def draw(self, surf, font, fmt=".5f"):
        pygame.draw.rect(surf, SLIDER_BG, self.rect, border_radius=3)
        ratio = (self.value - self.vmin) / (self.vmax - self.vmin)
        fw = int(self.rect.w * ratio)
        if fw > 0:
            pygame.draw.rect(surf, self.color,
                             pygame.Rect(self.rect.x, self.rect.y, fw, 6),
                             border_radius=3)
        kx = self.rect.x + int(self.rect.w * ratio)
        pygame.draw.circle(surf, self.color,     (kx, self.rect.centery), 9)
        pygame.draw.circle(surf, (240, 250, 255), (kx, self.rect.centery), 5)
        surf.blit(font.render(f"{self.label} : {self.value:{fmt}}", True, TEXT_COL),
                  (self.rect.x, self.rect.y - 20))

    def handle(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            kx = self.rect.x + int(self.rect.w *
                 (self.value - self.vmin) / (self.vmax - self.vmin))
            if math.hypot(event.pos[0]-kx, event.pos[1]-self.rect.centery) < 14:
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            r = (event.pos[0] - self.rect.x) / self.rect.w
            self.value = self.vmin + max(0.0, min(1.0, r)) * (self.vmax - self.vmin)


# ─── Error Graph ──────────────────────────────────────────────────────────────
class ErrorGraph(UIWidget):
    def __init__(self, x, y, w, h, label, color):
        super().__init__(x, y, w, h, label); self.color = color
        self.data = deque(maxlen=w)

    def push(self, v): self.data.append(v)

    def draw(self, surf, font):
        pygame.draw.rect(surf, GRAPH_BG, self.rect, border_radius=5)
        pygame.draw.rect(surf, (28, 36, 58), self.rect, 1, border_radius=5)
        mid = self.rect.centery
        pygame.draw.line(surf, (35, 45, 70), (self.rect.x, mid), (self.rect.right, mid))
        if len(self.data) > 1:
            pts = []
            for i, v in enumerate(self.data):
                px = self.rect.x + i
                py = mid - int(v / PLATE_SIZE * (self.rect.h // 2 - 3))
                pts.append((px, max(self.rect.y+2, min(self.rect.bottom-2, py))))
            pygame.draw.lines(surf, self.color, False, pts, 1)
        surf.blit(font.render(self.label, True, self.color),
                  (self.rect.x + 5, self.rect.y + 4))


# ─── ToggleMode (base) ────────────────────────────────────────────────────────
class ToggleMode:
    def __init__(self): self.active = False
    def toggle(self):   self.active = not self.active


# ─── Push Aimer ───────────────────────────────────────────────────────────────
class PushAimer(ToggleMode):
    """
    [RENAMED] ThrowAimer → PushAimer.
    [CLEANED] Removed dead methods: get_throw_params(), throw_angle_v.
    These were only used by the old 'throw ball in arc' logic which has been
    replaced by the realistic 'push ball on plate' impulse system.
    """
    def __init__(self):
        super().__init__()
        self.aiming   = False
        self.start_wx = 0.0; self.start_wy = 0.0; self.start_wz = 0.0
        self.start_sx = 0;   self.start_sy = 0
        self.end_sx   = 0;   self.end_sy   = 0

    def toggle(self):
        super().toggle()
        self.aiming = False

    def screen_to_world_flat(self, sx, sy, cam, cx, cy, z_world=0.0):
        az = math.radians(cam.azimuth)
        el = math.radians(cam.elevation)
        dx = (sx - cx - cam.pan_x) / cam.zoom
        dy = -(sy - cy - cam.pan_y) / cam.zoom
        ry = (dy + z_world * math.sin(el)) / max(math.cos(el), 0.01)
        wx =  dx * math.cos(az) - ry * math.sin(az)
        wy =  dx * math.sin(az) + ry * math.cos(az)
        return wx, wy


# ─── Orbit Controller ─────────────────────────────────────────────────────────
class OrbitController(ToggleMode):
    # [CLEANED] Removed pointless toggle() override (was just super().toggle())
    def __init__(self):
        super().__init__()
        self.angle = 0.0; self.radius = ORBIT_RADIUS; self.speed = ORBIT_SPEED

    def update(self, dt):
        if self.active: self.angle += self.speed * dt

    def get_target(self):
        if self.active:
            return (self.radius * math.cos(self.angle),
                    self.radius * math.sin(self.angle))
        return 0.0, 0.0


# ─── App ──────────────────────────────────────────────────────────────────────
class App:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption(
            "Ball & Plate ── 3D PID  |  Push + Orbit + Free Camera")
        self.clock = pygame.time.Clock()
        self.f_lg  = pygame.font.SysFont("consolas", 20, bold=True)
        self.f_md  = pygame.font.SysFont("consolas", 14)
        self.f_sm  = pygame.font.SysFont("consolas", 12)

        self.plate  = Plate()
        self.pid_x  = PIDController()
        self.pid_y  = PIDController()
        self.ball   = None
        self.drops  = 0
        self.cam    = Camera()
        self.cx     = SIM_W // 2
        self.cy     = HEIGHT // 2 + 40
        self.pusher = PushAimer()
        self.orbit  = OrbitController()

        # Drag state
        self.dragging_ball  = False
        self.drag_screen_x  = 0
        self.drag_screen_y  = 0
        self.drag_z         = 260.0
        self.drag_lift_base = 260.0
        self.drag_sy_base   = 0

        # [OPT] Per-frame tilt trig cache — set by _cache_tilt() each frame
        self._ctx = 1.0; self._stx = 0.0
        self._cty = 1.0; self._sty = 0.0

        # [OPT] Reusable alpha surface for plate fill — no new alloc each frame
        self._plate_surf = pygame.Surface((SIM_W, HEIGHT), pygame.SRCALPHA)

        px = SIM_W + 18
        pw = PANEL_W - 36

        self.sl_kp        = Slider(px, 82,  pw, "Kp",              0.001, 0.30,  0.08)
        self.sl_ki        = Slider(px, 148, pw, "Ki",              0.000, 0.005, 0.0005)
        self.sl_kd        = Slider(px, 214, pw, "Kd",              0.000, 1.0,   0.35)
        self.sl_bounce    = Slider(px, 290, pw, "Bounce",          0.0,   1.0,   RESTITUTION,
                                   color=(255, 180, 40))
        self.sl_push_max  = Slider(px, 366, pw, "Push Max Speed ", 30,    350,   150,
                                   color=PUSH_COL)
        self.sl_push_sens = Slider(px, 432, pw, "Push Sensitivity",0.5,   4.0,   1.5,
                                   color=PUSH_COL)
        self.sl_push_scl  = Slider(px, 498, pw, "Push Scale      ",0.5,   3.0,   1.2,
                                   color=(255, 140, 60))
        self.sl_orbit_r   = Slider(px, 574, pw, "Orbit Radius",   40,    190,   ORBIT_RADIUS,
                                   color=ORBIT_COL)
        self.sl_orbit_s   = Slider(px, 630, pw, "Orbit Speed ",   0.1,   3.0,   ORBIT_SPEED,
                                   color=ORBIT_COL)
        self.gr_x = ErrorGraph(px, 688, pw, 36, "Error X", (0,   200, 255))
        self.gr_y = ErrorGraph(px, 730, pw, 36, "Error Y", (255, 165, 0))
        self.ttx = 0.0; self.tty = 0.0
        self._all_sliders = [
            self.sl_kp, self.sl_ki, self.sl_kd, self.sl_bounce,
            self.sl_push_max, self.sl_push_sens, self.sl_push_scl,
            self.sl_orbit_r, self.sl_orbit_s,
        ]

    # ── Projection helpers ────────────────────────────────────────────────────
    def _proj(self, x, y, z):
        return self.cam.project(x, y, z, self.cx, self.cy)

    def _cache_tilt(self):
        """[OPT] Precompute plate tilt sin/cos once per frame."""
        tx_r = math.radians(self.plate.tilt_x)
        ty_r = math.radians(self.plate.tilt_y)
        self._ctx = math.cos(tx_r); self._stx = math.sin(tx_r)
        self._cty = math.cos(ty_r); self._sty = math.sin(ty_r)

    def _tp(self, x, y, z):
        """Rotate local plate coords using cached trig."""
        y2 =  y * self._ctx - z * self._stx
        z2 =  y * self._stx + z * self._ctx
        x3 =  x * self._cty + z2 * self._sty
        z3 = -x * self._sty + z2 * self._cty
        return x3, y2, z3

    def _proj_tp(self, x, y, z):
        return self._proj(*self._tp(x, y, z))

    def _screen_to_world(self, sx, sy, z_world=0.0):
        return self.pusher.screen_to_world_flat(
            sx, sy, self.cam, self.cx, self.cy, z_world=z_world)

    # ── Drag helpers ──────────────────────────────────────────────────────────
    def _start_drag(self, mx, my):
        self.dragging_ball  = True
        self.drag_screen_x  = mx; self.drag_screen_y  = my
        self.drag_z         = 260.0; self.drag_lift_base = 260.0
        self.drag_sy_base   = my
        wx, wy = self._screen_to_world(mx, my, z_world=self.drag_z)
        self.ball = Ball(wx, wy, z=self.drag_z)
        self.ball.alive = True
        self.pid_x.reset(); self.pid_y.reset()

    def _update_drag(self, mx, my):
        if not self.dragging_ball: return
        self.drag_screen_x = mx; self.drag_screen_y = my
        dy = self.drag_sy_base - my
        self.drag_z = max(0.0, min(800.0, self.drag_lift_base + dy * 2.5))
        wx, wy = self._screen_to_world(mx, my, z_world=self.drag_z)
        if self.ball is not None:
            self.ball.x = wx; self.ball.y = wy; self.ball.z = self.drag_z
            self.ball.vx = 0.0; self.ball.vy = 0.0; self.ball.vz = 0.0
            self.ball.on_plate = False; self.ball.alive = True

    def _release_drag(self):
        if not self.dragging_ball: return
        self.dragging_ball = False
        if self.ball is not None:
            self.ball.alive = True; self.ball.on_plate = False
        self.pid_x.reset(); self.pid_y.reset()

    # ── Draw plate ────────────────────────────────────────────────────────────
    def _draw_plate(self):
        P = PLATE_SIZE; SEGS = 64
        circle_pts = [
            self._proj_tp(P * math.cos(2*math.pi*i/SEGS),
                          P * math.sin(2*math.pi*i/SEGS), 0)
            for i in range(SEGS)]

        # [OPT] Clear & redraw cached surface — no new 900x780 alloc each frame
        self._plate_surf.fill((0, 0, 0, 0))
        pygame.draw.polygon(self._plate_surf, (20, 140, 185, 140), circle_pts)
        self.screen.blit(self._plate_surf, (0, 0))

        steps = 7
        for i in range(-steps, steps+1):
            f = i / steps * P
            disc = P*P - f*f
            if disc <= 0: continue
            half = math.sqrt(disc)
            pygame.draw.line(self.screen, GRID_COL,
                             self._proj_tp(-half, f, 0), self._proj_tp(half, f, 0), 1)
            pygame.draw.line(self.screen, GRID_COL,
                             self._proj_tp(f, -half, 0), self._proj_tp(f, half, 0), 1)
        pygame.draw.lines(self.screen, PLATE_EDGE, True, circle_pts, 2)

        if self.orbit.active:
            r = self.orbit.radius
            op = [self._proj_tp(r*math.cos(2*math.pi*i/SEGS),
                                r*math.sin(2*math.pi*i/SEGS), 1)
                  for i in range(SEGS)]
            # [FIX] Use dim RGB color instead of RGBA-in-color (alpha ignored
            # on non-SRCALPHA surface)
            dim = (ORBIT_COL[0]//2, ORBIT_COL[1]//2, ORBIT_COL[2]//2)
            for i in range(0, SEGS, 2):
                pygame.draw.line(self.screen, dim, op[i], op[(i+1)%SEGS], 1)
            tx, ty = self.orbit.get_target()
            pygame.draw.circle(self.screen, ORBIT_COL, self._proj_tp(tx, ty, 2), 6, 2)

        cp = self._proj_tp(0, 0, 3)
        pygame.draw.line(self.screen, (255,255,80), (cp[0]-10,cp[1]),(cp[0]+10,cp[1]),1)
        pygame.draw.line(self.screen, (255,255,80), (cp[0],cp[1]-6), (cp[0],cp[1]+6), 1)
        o = self._proj(0,0,0)
        pygame.draw.line(self.screen,(200,60, 60), o, self._proj(60,0, 0), 2)
        pygame.draw.line(self.screen,(60, 200,60), o, self._proj(0, 60,0), 2)
        pygame.draw.line(self.screen,(60, 60, 200),o, self._proj(0, 0, 60),2)
        self.screen.blit(self.f_sm.render("X",True,(200,60, 60)),self._proj(65,0, 0))
        self.screen.blit(self.f_sm.render("Y",True,(60, 200,60)), self._proj(0, 65,0))
        self.screen.blit(self.f_sm.render("Z",True,(60, 100,200)),self._proj(0, 0, 65))

    # ── Draw legs ─────────────────────────────────────────────────────────────
    def _draw_legs(self):
        """[OPT] Uses cached trig from _cache_tilt(); foot surface created
        once per frame (not once per leg)."""
        ctx = self._ctx; stx = self._stx
        cty = self._cty; sty = self._sty

        thick_outer = max(4, int(7 * self.cam.zoom))
        thick_inner = max(3, int(5 * self.cam.zoom))
        thick_shell = max(2, int(3 * self.cam.zoom))
        collar_r    = max(4, int(6 * self.cam.zoom))
        joint_r     = max(4, int(6 * self.cam.zoom))
        foot_w      = max(8, int(14 * self.cam.zoom))
        foot_h      = max(3, int(5 * self.cam.zoom))

        # [OPT] Build foot surface once, reuse for all 3 legs
        foot_surf = pygame.Surface((foot_w*2, foot_h*2), pygame.SRCALPHA)
        pygame.draw.ellipse(foot_surf, (*LEG_FOOT_COL, 220), foot_surf.get_rect())
        pygame.draw.ellipse(foot_surf, (120,200,255,160), foot_surf.get_rect(), 1)

        for i in range(LEG_COUNT):
            ba  = 2*math.pi*i/LEG_COUNT + math.pi/6
            cos_a = math.cos(ba); sin_a = math.sin(ba)
            axl = LEG_ATTACH_R * cos_a
            ayl = LEG_ATTACH_R * sin_a
            ay2 =  ayl * ctx
            az2 =  ayl * stx
            ax3 =  axl * cty + az2 * sty
            az3 = -axl * sty + az2 * cty

            fx = LEG_FOOT_R*cos_a; fy = LEG_FOOT_R*sin_a; fz = LEG_BASE_Z
            ro = 0.52
            mxw = fx+(ax3-fx)*ro; myw = fy+(ay2-fy)*ro; mzw = fz+(az3-fz)*ro

            pf  = self._proj(fx,  fy,  fz)
            pm  = self._proj(mxw, myw, mzw)
            pa  = self._proj(ax3, ay2, az3)

            pygame.draw.line(self.screen, LEG_COL_OUTER,  pf, pm, thick_outer)
            pygame.draw.line(self.screen, (20,220,255),    pf, pm, thick_shell)
            pygame.draw.line(self.screen, LEG_COL_INNER,  pm, pa, thick_inner)
            pygame.draw.line(self.screen, (180,255,240),   pm, pa, thick_shell)

            for b in range(1, 4):
                t = b/4
                bx=int(pm[0]+(pa[0]-pm[0])*t); by=int(pm[1]+(pa[1]-pm[1])*t)
                pl = max(2, int(4*self.cam.zoom))
                dxs=pa[0]-pm[0]; dys=pa[1]-pm[1]; sl=max(math.hypot(dxs,dys),1)
                pxn=-dys/sl*pl; pyn=dxs/sl*pl
                pygame.draw.line(self.screen,(0,200,170),
                                 (int(bx-pxn),int(by-pyn)),(int(bx+pxn),int(by+pyn)),1)

            for b in range(1, 3):
                t = b/3
                bx=int(pf[0]+(pm[0]-pf[0])*t); by=int(pf[1]+(pm[1]-pf[1])*t)
                pl=max(3,int(5*self.cam.zoom))
                dxs=pm[0]-pf[0]; dys=pm[1]-pf[1]; sl=max(math.hypot(dxs,dys),1)
                pxn=-dys/sl*pl; pyn=dxs/sl*pl
                pygame.draw.line(self.screen,(0,150,190),
                                 (int(bx-pxn),int(by-pyn)),(int(bx+pxn),int(by+pyn)),2)

            pygame.draw.circle(self.screen,(20,40,60),   pm, collar_r+2)
            pygame.draw.circle(self.screen,LEG_COL_OUTER,pm, collar_r)
            pygame.draw.circle(self.screen,(120,240,255),pm, max(2,collar_r-2),1)
            pygame.draw.circle(self.screen,(10,30,50),   pa, joint_r+2)
            pygame.draw.circle(self.screen,LEG_JOINT_COL,pa, joint_r)
            pygame.draw.circle(self.screen,(255,255,180),pa, max(2,joint_r-2),1)
            self.screen.blit(foot_surf, (pf[0]-foot_w, pf[1]-foot_h))

    # ── Draw ball ─────────────────────────────────────────────────────────────
    def _draw_ball(self):
        b = self.ball
        if b is None or not b.alive: return

        # [FIX] Trail uses RGB brightness fading — the old code passed alpha as
        # 4th color component to pygame.draw.line on a non-SRCALPHA surface,
        # where alpha is silently ignored. The trail was fully opaque all along.
        if len(b.trail) > 1:
            tpts = [self._proj(tx,ty,tz) for tx,ty,tz in b.trail]
            n = len(tpts)
            for i in range(1, n):
                t = i / n
                pygame.draw.line(
                    self.screen,
                    (int(80+175*t), int(20+40*t), int(20+40*t)),
                    tpts[i-1], tpts[i], 1)

        bx3,by3,bz3 = self._tp(b.x, b.y, b.z + BALL_RADIUS*0.5)
        sx, sy = self._proj(bx3, by3, bz3)
        r = max(5, int(BALL_RADIUS * self.cam.zoom))

        shx,shy   = self._proj_tp(b.x, b.y, 1)
        hab       = max(0, b.z)
        sha       = max(20, 100 - int(hab*0.3))
        sw        = int(r*3*(1+hab*0.003)); sh2 = max(2, int(sw*0.3))
        ss        = pygame.Surface((sw, sh2), pygame.SRCALPHA)
        pygame.draw.ellipse(ss, (0,0,0,sha), ss.get_rect())
        self.screen.blit(ss, (shx-sw//2, shy-sh2//2))

        if b.bounce_flash > 0:
            fr = r + int(b.bounce_flash/120*8)
            fa = int(b.bounce_flash/120*180)
            fs = pygame.Surface((fr*2+4, fr*2+4), pygame.SRCALPHA)
            pygame.draw.circle(fs, (*BOUNCE_COL,fa), (fr+2,fr+2), fr)
            self.screen.blit(fs, (sx-fr-2, sy-fr-2))

        bs = pygame.Surface((r*4, r*4), pygame.SRCALPHA)
        pygame.draw.circle(bs, BALL_COL,   (r*2, r*2), r)
        pygame.draw.circle(bs, BALL_SHINE, (r*2-r//3, r*2-r//3), max(2,r//3))
        self.screen.blit(bs, (sx-r*2, sy-r*2))

        if self.dragging_ball:
            gpt = self._proj(b.x, b.y, 0)
            pygame.draw.line(self.screen, (80,220,80), (sx,sy), gpt, 1)
            pygame.draw.circle(self.screen, (80,220,80), gpt, 4, 1)
            self.screen.blit(
                self.f_md.render(f"Z = {b.z:.0f}", True, PUSH_COL),
                (sx+r+6, sy-8))

    # ── Draw push aimer ───────────────────────────────────────────────────────
    def _draw_aimer(self):
        if not self.pusher.active: return
        mx, my = pygame.mouse.get_pos()
        if mx >= SIM_W: return
        if not self.pusher.aiming:
            pygame.draw.line(self.screen, PUSH_COL, (mx-14,my),(mx+14,my), 1)
            pygame.draw.line(self.screen, PUSH_COL, (mx,my-14),(mx,my+14), 1)
            pygame.draw.circle(self.screen, PUSH_COL, (mx,my), 10, 1)
            self.screen.blit(
                self.f_sm.render("Click & drag to push", True, PUSH_COL),
                (mx+14, my-8))
        else:
            sx2,sy2 = self.pusher.start_sx, self.pusher.start_sy
            ex, ey  = self.pusher.end_sx,   self.pusher.end_sy
            ddx,ddy = ex-sx2, ey-sy2
            length  = math.hypot(ddx, ddy)
            push_power = (min(length * self.sl_push_sens.value,
                              self.sl_push_max.value)
                          * self.sl_push_scl.value)
            pygame.draw.line(self.screen, (60,200,80), (sx2,sy2),(ex,ey), 2)
            pygame.draw.circle(self.screen, PUSH_COL, (sx2,sy2), 7, 2)
            if length > 5:
                ndx,ndy = ddx/length, ddy/length
                tx2,ty2 = int(ex), int(ey)
                perp = (-ndy, ndx)
                p1=(int(tx2-ndx*14+perp[0]*7),int(ty2-ndy*14+perp[1]*7))
                p2=(int(tx2-ndx*14-perp[0]*7),int(ty2-ndy*14-perp[1]*7))
                pygame.draw.polygon(self.screen, PUSH_COL, [(tx2,ty2),p1,p2])
            self.screen.blit(
                self.f_sm.render(
                    f"Force: {push_power:.0f}  (Max: {self.sl_push_max.value:.0f})",
                    True, PUSH_COL),
                (sx2+12, sy2-22))

    # ── Draw panel ────────────────────────────────────────────────────────────
    def _draw_panel(self):
        pygame.draw.rect(self.screen, PANEL_BG, pygame.Rect(SIM_W,0,PANEL_W,HEIGHT))
        pygame.draw.line(self.screen, ACCENT, (SIM_W,0),(SIM_W,HEIGHT), 2)
        self.screen.blit(self.f_lg.render("BALL & PLATE",True,ACCENT),(SIM_W+18,12))
        self.screen.blit(
            self.f_sm.render("3D PID + PUSH + ORBIT + FREE CAM",True,(60,100,140)),
            (SIM_W+18,36))
        for sl in (self.sl_kp, self.sl_ki, self.sl_kd):
            sl.draw(self.screen, self.f_md)
        self.sl_bounce.draw(self.screen, self.f_md, fmt=".2f")

        pygame.draw.rect(self.screen,(14,22,38),
                         pygame.Rect(SIM_W+10,344,PANEL_W-20,178),border_radius=5)
        plbl = ("PUSH  MODE [U]  ●ON" if self.pusher.active
                else "PUSH  MODE [U]  ○off")
        pcol = PUSH_COL if self.pusher.active else (60,100,80)
        self.screen.blit(self.f_md.render(plbl,True,pcol),(SIM_W+18,348))
        self.sl_push_max.draw( self.screen, self.f_md, fmt=".0f")
        self.sl_push_sens.draw(self.screen, self.f_md, fmt=".2f")
        self.sl_push_scl.draw( self.screen, self.f_md, fmt=".2f")

        pygame.draw.rect(self.screen,(18,14,38),
                         pygame.Rect(SIM_W+10,554,PANEL_W-20,106),border_radius=5)
        olbl = ("ORBIT MODE  [O]  ●ON" if self.orbit.active
                else "ORBIT MODE  [O]  ○off")
        ocol = ORBIT_COL if self.orbit.active else (80,60,100)
        self.screen.blit(self.f_md.render(olbl,True,ocol),(SIM_W+18,558))
        self.sl_orbit_r.draw(self.screen, self.f_md, fmt=".0f")
        self.sl_orbit_s.draw(self.screen, self.f_md, fmt=".2f")

        self.gr_x.draw(self.screen, self.f_sm)
        self.gr_y.draw(self.screen, self.f_sm)

        y = HEIGHT-124
        pygame.draw.rect(self.screen,(18,22,42),
                         pygame.Rect(SIM_W+10,y,PANEL_W-20,50),border_radius=6)
        self.screen.blit(self.f_md.render("PLATE TILT",True,(60,100,140)),(SIM_W+18,y+4))
        self.screen.blit(
            self.f_md.render(f"Tilt X : {self.plate.tilt_x:+6.2f}°",True,(0,200,255)),
            (SIM_W+18, y+22))
        self.screen.blit(
            self.f_md.render(f"Tilt Y : {self.plate.tilt_y:+6.2f}°",True,(255,165,0)),
            (SIM_W+170, y+22))

        y2 = HEIGHT-68
        pygame.draw.rect(self.screen,(18,22,42),
                         pygame.Rect(SIM_W+10,y2,PANEL_W-20,34),border_radius=6)
        col = DANGER if self.drops > 0 else SUCCESS
        self.screen.blit(self.f_md.render("DROPS",True,(60,100,140)),(SIM_W+18,y2+2))
        self.screen.blit(self.f_lg.render(str(self.drops),True,col),(SIM_W+18,y2+16))
        if self.ball:
            self.screen.blit(
                self.f_md.render("BOUNCES",True,(60,100,140)),(SIM_W+110,y2+2))
            self.screen.blit(
                self.f_lg.render(str(self.ball.bounces),True,BOUNCE_COL),
                (SIM_W+110,y2+16))
        self.screen.blit(
            self.f_sm.render(
                f"CAM  Az:{self.cam.azimuth:.0f}°  "
                f"El:{self.cam.elevation:.0f}°  Z:{self.cam.zoom:.2f}",
                True,(60,90,120)),
            (SIM_W+18, HEIGHT-30))
        hints = ["[U] Push  [O] Orbit  [R] Reset  [C] Cam",
                 "RMB drag=rotate  Shift+RMB=pan  Scroll=zoom"]
        for i,h in enumerate(hints):
            self.screen.blit(self.f_sm.render(h,True,(35,55,80)),
                             (SIM_W+18, HEIGHT-54+i*14))

    # ── Main loop ─────────────────────────────────────────────────────────────
    def run(self):
        prev = pygame.time.get_ticks()
        while True:
            now  = pygame.time.get_ticks()
            dt   = min((now - prev) / 1000.0, 0.04)
            prev = now

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit(); sys.exit()
                self.cam.handle(event)

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        self.ball = None
                        self.plate.tilt_x = self.plate.tilt_y = 0
                        self.pid_x.reset(); self.pid_y.reset()
                        self.drops = 0; self.orbit.angle = 0.0
                        self.dragging_ball = False
                    if event.key == pygame.K_c:
                        self.cam.reset()
                    if event.key == pygame.K_u:   # [CHANGED] T → U
                        self.pusher.toggle()
                        self.dragging_ball = False
                    if event.key == pygame.K_o:
                        self.orbit.toggle()
                        self.pid_x.reset(); self.pid_y.reset()

                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    mx, my = event.pos
                    if mx < SIM_W:
                        if self.pusher.active:
                            # Place ball on plate if none exists
                            if self.ball is None or not self.ball.alive:
                                wx,wy = self._screen_to_world(mx, my, 0.0)
                                dist  = math.hypot(wx, wy)
                                if dist > PLATE_SIZE * 0.82:
                                    s = PLATE_SIZE*0.82/max(dist,1e-9)
                                    wx *= s; wy *= s
                                bz = wx*self._sty - wy*self._stx
                                self.ball = Ball(wx, wy, z=bz)
                                self.ball.on_plate = True; self.ball.alive = True
                                self.pid_x.reset(); self.pid_y.reset()
                            # Snap aimer origin to ball's screen coords
                            bx3,by3,bz3 = self._tp(
                                self.ball.x, self.ball.y,
                                self.ball.z + BALL_RADIUS*0.5)
                            bsx,bsy = self._proj(bx3,by3,bz3)
                            self.pusher.aiming   = True
                            self.pusher.start_sx = bsx; self.pusher.start_sy = bsy
                            self.pusher.end_sx   = mx;  self.pusher.end_sy   = my
                            self.pusher.start_wx = self.ball.x
                            self.pusher.start_wy = self.ball.y
                            self.pusher.start_wz = self.ball.z
                        else:
                            self._start_drag(mx, my)

                if event.type == pygame.MOUSEMOTION:
                    mx, my = event.pos
                    if self.pusher.aiming:
                        self.pusher.end_sx, self.pusher.end_sy = mx, my
                    if self.dragging_ball:
                        self._update_drag(mx, my)

                if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    if self.dragging_ball:
                        self._release_drag()
                    elif self.pusher.aiming:
                        self.pusher.aiming = False
                        if self.ball is not None and self.ball.alive:
                            ddx = self.pusher.end_sx - self.pusher.start_sx
                            ddy = self.pusher.end_sy - self.pusher.start_sy
                            length = math.hypot(ddx, ddy)
                            if length > 4:
                                az = math.radians(self.cam.azimuth)
                                el = math.radians(self.cam.elevation)
                                ndx,ndy = ddx/length, ddy/length
                                wxd = (ndx*math.cos(az)
                                       - ndy*math.sin(az)/max(math.cos(el),0.01))
                                wyd = (ndx*math.sin(az)
                                       + ndy*math.cos(az)/max(math.cos(el),0.01))
                                ln  = math.hypot(wxd, wyd) or 1.0
                                push_power = (min(length*self.sl_push_sens.value,
                                                  self.sl_push_max.value)
                                              * self.sl_push_scl.value)
                                self.ball.vx      += wxd/ln * push_power
                                self.ball.vy      += wyd/ln * push_power
                                self.ball.vz       = 0.0
                                self.ball.on_plate = True
                                # [OPT] Soft-reset integral to reduce overshoot
                                # on new trajectory after push
                                self.pid_x._integral *= 0.35
                                self.pid_y._integral *= 0.35

                for sl in self._all_sliders:
                    sl.handle(event)

            # ── Sync gains ────────────────────────────────────────────────────
            for pid in (self.pid_x, self.pid_y):
                pid.kp = self.sl_kp.value
                pid.ki = self.sl_ki.value
                pid.kd = self.sl_kd.value

            # [FIX] Removed 'global RESTITUTION' antipattern — value is now
            # passed directly as a parameter into Ball.update()
            restitution = self.sl_bounce.value

            self.orbit.radius = self.sl_orbit_r.value
            self.orbit.speed  = self.sl_orbit_s.value
            self.orbit.update(dt)
            target_x, target_y = self.orbit.get_target()

            if self.ball and self.ball.alive and not self.dragging_ball:
                err_x = self.ball.x - target_x
                err_y = self.ball.y - target_y
                # [OPT] Raised off-plate weight 0.5→0.7 for faster recovery
                weight     = 1.0 if self.ball.on_plate else 0.7
                self.tty   = -self.pid_x.update(err_x, dt) * weight
                self.ttx   = -self.pid_y.update(err_y, dt) * weight
                self.gr_x.push(err_x)
                self.gr_y.push(err_y)
            else:
                self.ttx *= 0.93; self.tty *= 0.93

            self.plate.update(self.ttx, self.tty, dt)

            if self.ball and not self.dragging_ball:
                self.ball.update(self.plate.tilt_x, self.plate.tilt_y,
                                 dt, plate=self.plate, restitution=restitution)
                if not self.ball.alive:
                    self.drops += 1; self.ball = None

            # ── Render ────────────────────────────────────────────────────────
            self.screen.fill(BG)
            for gx in range(0, SIM_W, 44):
                pygame.draw.line(self.screen,(13,16,26),(gx,0),(gx,HEIGHT))
            for gy in range(0, HEIGHT, 44):
                pygame.draw.line(self.screen,(13,16,26),(0,gy),(SIM_W,gy))

            # [OPT] Cache trig once for entire render pass
            self._cache_tilt()

            self._draw_legs()
            self._draw_plate()
            self._draw_ball()
            self._draw_aimer()
            self._draw_panel()
            self.screen.blit(
                self.f_lg.render(
                    "BALL & PLATE  ──  3D PID  |  PUSH & ORBIT & FREE CAM",
                    True,(25,45,75)),
                (14,10))
            pygame.display.flip()
            self.clock.tick(FPS)


if __name__ == "__main__":
    App().run()