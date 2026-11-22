# Sweep for contacts, then narrow the beam to get a firing bearing.
# Clunkyard guide: https://clunkyard.com/sdk/riscv

.include "clunkyard.inc"

# Load a double constant from .rodata. Clobbers t6.
.macro fload freg, label
    la t6, \label
    fld \freg, 0(t6)
.endm

# ---- Constants ----

.section .rodata
.balign 8
ZERO:          .double 0.0
HALF:          .double 0.5
ONE:           .double 1.0
TWO:           .double 2.0
SEARCH_TURN:   .double 0.33     # enough to turn against the brake in the force rulesets
SEARCH_ARC:    .double 40.0     # degrees: the search beam
MIN_ARC:       .double 6.0      # degrees: the narrowest the beam holds
PRECISE:       .double 12.0     # units across the line of sight a shot may be off by
BRAKE:         .double 0.2
FAR:           .double 350.0
NEAR:          .double 200.0
CLOSE_IN:      .double 0.7
BACK_OFF:      .double -0.4
CRUISE:        .double 0.2
GAIN:          .double 45.0
TIGHT_GAIN:    .double 30.0
ON_TARGET:     .double 3.0      # degrees off the target the turret may be
SPARE:         .double 10.0     # battery to keep after a shot
DEGREES:       .double 57.3     # degrees in a radian, near enough
LOOK_ARC:      .double 30.0
LOOK_STEP:     .double 8.0
SWEEP_STEP:    .double 36.0
SWEEP_PITCH:   .double 35.0
CUE_SPEED:     .double 0.6
WANDER:        .double 0.5
STORM_EDGE:    .double -60.0
ZONE_EDGE:     .double 60.0
QUARTER_TURN:  .double 90.0
HOME_SPEED:    .double 0.8
BEAM_HALF:     .double 30.0     # half of the 60 units the held beam spans
ATAN_FIT:      .double 0.28125
HALF_PI:       .double 1.5707963
DEGREES_EXACT: .double 57.29578

# ---- SDK results and robot state ----

.section .bss
.balign 8
NUMBER:   .zero 8                     # one double a call has just written
WHOLE:    .zero 8                     # one whole number a call has just written
ME:       .zero CY_VEC3_SIZE          # this robot's position
LAST:     .zero CY_VEC3_SIZE          # where a robot was last seen
STILL:    .zero CY_VEC3_SIZE          # a zero vector, never written
GRAVITY:  .zero CY_VEC3_SIZE
TARGET:   .zero CY_CONTACT_SIZE
AIM:      .zero CY_SOLUTION_SIZE
NOW:      .zero CY_DIRECTION_SIZE
CUE:      .zero CY_CUE_SIZE
SCAN:     .zero CY_SCAN_SIZE
STORM:    .zero CY_STORM_SIZE
ZONE:     .zero CY_ZONE_SIZE
CUES:     .zero CY_CONTACT_SIZE * 8   # room for 8 contacts
.balign 4
dodge_until: .zero 4                  # the tick the dodge ends
dodges:      .zero 4                  # how many dodges so far
free:        .zero 4                  # 1 flying free: robots may be above or below
forces:      .zero 4                  # 1 in the force rulesets: push the hull, not steer it
last_slot:   .zero 4                  # the robot seen most recently, 0 for none yet

.text

# ---- Tactic helpers ----

# angle_diff(fa0 target, fa1 current) -> fa0: target - current, in -180..180.
angle_diff:
    la a0, NUMBER
    cy_angles_angle_diff
    la t0, NUMBER
    fld fa0, 0(t0)
    ret

# bearing_to(a0 from, a1 to) -> fa0: the heading from one point to another.
bearing_to:
    la a2, NUMBER
    cy_angles_bearing_to
    la t0, NUMBER
    fld fa0, 0(t0)
    ret

# elevation_to(a0 from, a1 to) -> fa0: the pitch from one point to another.
elevation_to:
    la a2, NUMBER
    cy_angles_elevation_to
    la t0, NUMBER
    fld fa0, 0(t0)
    ret

# unit(fa0) -> fa0, held between -1 and 1.
unit:
    fload ft0, ONE
    fmin.d fa0, fa0, ft0
    fneg.d ft0, ft0
    fmax.d fa0, fa0, ft0
    ret

# hold_arc(fa0 range) -> fa0: the angle, in degrees, of a beam 60 units
# across at this range: 2 * atan(30 / range), by a rational fit that costs
# less than a series.
hold_arc:
    fload ft0, BEAM_HALF
    fdiv.d ft0, ft0, fa0            # x = 30 / range
    fload ft1, ONE
    flt.d t0, ft1, ft0              # t0 = 1 when x > 1: fit 1/x instead
    beqz t0, 1f
    fdiv.d ft0, ft1, ft0
1:  fmul.d ft2, ft0, ft0            # angle = x / (1 + 0.28125 x^2)
    fload ft3, ATAN_FIT
    fmadd.d ft2, ft2, ft3, ft1
    fdiv.d ft2, ft0, ft2
    beqz t0, 2f
    fload ft3, HALF_PI              # atan(x) = pi/2 - atan(1/x)
    fsub.d ft2, ft3, ft2
2:  fload ft3, TWO
    fmul.d ft2, ft2, ft3
    fload ft3, DEGREES_EXACT
    fmul.d fa0, ft2, ft3
    ret

# scan(fa0 heading, fa1 arc, fa2 pitch): points the scanner.
scan:
    cy_scanner_heading
    fmv.d fa0, fa1
    cy_scanner_arc
    fmv.d fa0, fa2
    cy_scanner_pitch
    ret

# steer(fa0 throttle, fa1 heading, fa2 elevation, fa3 gain, fa4 my heading,
# fa5 my pitch): drives towards a heading and an elevation, turning harder
# the smaller the gain.
steer:
    addi sp, sp, -32
    sw ra, 28(sp)
    fsd fs0, 0(sp)
    fsd fs1, 8(sp)
    fsd fs2, 16(sp)
    fmv.d fs0, fa2                  # the calls below may change fa registers,
    fmv.d fs1, fa3                  # so keep what is needed in saved ones
    fmv.d fs2, fa5
    cy_drive_throttle               # fa0 is the throttle already
    fmv.d fa0, fa1
    fmv.d fa1, fa4
    call angle_diff                 # how far off the heading
    fdiv.d fa0, fa0, fs1
    call unit
    cy_drive_turn
    fsub.d fa0, fs0, fs2            # how far off the elevation
    fdiv.d fa0, fa0, fs1
    call unit
    cy_drive_pitch_turn
    fld fs0, 0(sp)
    fld fs1, 8(sp)
    fld fs2, 16(sp)
    lw ra, 28(sp)
    addi sp, sp, 32
    ret

# ---- the entry points ----

.globl clunkyard_start
clunkyard_start:
    la a0, GRAVITY
    cy_arena_gravity
    la a0, WHOLE
    cy_match_is_3d                  # 1 in a 3-D match
    la t0, WHOLE
    lw t1, 0(t0)
    la t2, GRAVITY
    fld ft0, CY_VEC3_Z(t2)
    fcvt.d.w ft1, zero
    feq.d t3, ft0, ft1              # no gravity
    and t1, t1, t3                  # flying free: 3-D and no gravity
    la t0, free
    sw t1, 0(t0)
    la a0, WHOLE
    cy_match_control
    la t0, WHOLE
    lw t1, 0(t0)
    addi t1, t1, -CY_CONTROL_FORCES
    seqz t1, t1
    la t0, forces
    sw t1, 0(t0)
    li a0, 0
    ret

# The turn. The values it keeps across calls live in saved registers:
#   s0 tick      s1 the record being read
#   fs0 heading  fs1 pitch     fs2 range   fs3 bearing  fs4 elevation
#   fs5 power    fs6 sensed bearing error
.globl clunkyard_turn
clunkyard_turn:
    addi sp, sp, -80
    sw ra, 76(sp)
    sw s0, 72(sp)
    sw s1, 68(sp)
    fsd fs0, 0(sp)
    fsd fs1, 8(sp)
    fsd fs2, 16(sp)
    fsd fs3, 24(sp)
    fsd fs4, 32(sp)
    fsd fs5, 40(sp)
    fsd fs6, 48(sp)

    la a0, ME
    cy_me_position
    la a0, NUMBER
    cy_me_heading
    la t0, NUMBER
    fld fs0, 0(t0)
    # A call that answers CY_NONE (pitch, in 2-D) leaves its result as it
    # was, so clear the double first: two zero words.
    sw zero, 0(t0)
    sw zero, 4(t0)
    la a0, NUMBER
    cy_me_pitch
    fld fs1, 0(t0)                  # an ecall keeps t0
    la a0, WHOLE
    cy_clock_tick
    la t0, WHOLE
    lw s0, 0(t0)
    fload fa0, BRAKE
    cy_drive_brake

    la a0, TARGET
    cy_sensors_nearest
    bnez a0, no_contact             # CY_OK is 0: anything else, no contact

    # ---- a robot in the scan: hold it, lead it, fire ----
    la s1, TARGET
    lw t1, CY_CONTACT_SLOT(s1)
    la t0, last_slot
    sw t1, 0(t0)
    fld fs2, CY_CONTACT_RANGE(s1)
    fld fs3, CY_CONTACT_BEARING(s1)
    fld fs4, CY_CONTACT_ELEVATION(s1)
    fld fs6, CY_CONTACT_BEARING_ERROR(s1)
    # Close in from afar, back off when near, cruise between.
    fload fa0, CLOSE_IN
    fload ft0, FAR
    flt.d t1, ft0, fs2
    bnez t1, 1f
    fload fa0, CRUISE
    fload ft0, NEAR
    flt.d t1, fs2, ft0
    beqz t1, 1f
    fload fa0, BACK_OFF
1:  fmv.d fa1, fs3
    fmv.d fa2, fs4
    fload fa3, GAIN
    fmv.d fa4, fs0
    fmv.d fa5, fs1
    call steer
    # Halve the beam each tick until it is 60 units across at the target:
    # narrow enough for a precise bearing, wide enough to keep it in. Every
    # 40 ticks widen it for a tick, so a nearer robot can be found.
    la a0, SCAN
    cy_scanner_last
    li t1, 40
    rem t1, s0, t1
    fload fa1, SEARCH_ARC
    beqz t1, 2f
    fmv.d fa0, fs2
    call hold_arc
    fload ft0, MIN_ARC
    fmax.d ft1, fa0, ft0            # max(6, hold_arc(range))
    la t0, SCAN
    fld ft2, CY_SCAN_ARC(t0)
    fload ft3, TWO
    fdiv.d ft2, ft2, ft3
    fload ft3, SEARCH_ARC
    fmin.d ft2, ft2, ft3            # min(40, last arc / 2)
    fmax.d fa1, ft1, ft2
2:  fmv.d fa0, fs3
    fmv.d fa2, fs4
    call scan
    # The heaviest shot that still covers the range in about a second; if
    # none does, NUMBER keeps the 1 written first.
    fload ft0, ONE
    la a0, NUMBER
    fsd ft0, 0(a0)
    fmv.d fa0, fs2
    fmv.d fa1, ft0
    cy_gun_power_for_flight_time
    la t0, NUMBER
    fld fs5, 0(t0)
    # An exact contact is led: aimed where it will be if it holds its
    # course. A noisy one is aimed at as sensed.
    fcvt.d.w ft0, zero
    feq.d t1, fs6, ft0
    beqz t1, 3f
    la a0, TARGET
    fmv.d fa0, fs5
    la a1, AIM
    cy_gun_lead
    bnez a0, 3f
    la t0, AIM
    fld fs3, CY_SOLUTION_HEADING(t0)
    fld fs4, CY_SOLUTION_PITCH(t0)
3:  fmv.d fa0, fs3
    cy_gun_aim_heading
    fmv.d fa0, fs4
    cy_gun_aim_pitch
    la a0, NOW
    cy_gun_aim_now
    # Fire once one reading is precise (its error across the line of sight
    # under 12 units), the turret is within 3 degrees and the battery covers
    # the shot with 10 to spare.
    fmul.d ft0, fs2, fs6
    fload ft1, DEGREES
    fdiv.d ft0, ft0, ft1
    fload ft1, PRECISE
    flt.d t1, ft0, ft1
    beqz t1, contact_done
    fmv.d fa0, fs3
    la t0, NOW
    fld fa1, CY_DIRECTION_HEADING(t0)
    call angle_diff
    fabs.d fa0, fa0
    fload ft1, ON_TARGET
    flt.d t1, fa0, ft1
    beqz t1, contact_done
    la t0, NOW
    fld ft0, CY_DIRECTION_PITCH(t0)
    fsub.d ft0, fs4, ft0
    fabs.d ft0, ft0
    flt.d t1, ft0, ft1
    beqz t1, contact_done
    fmv.d fa0, fs5
    fload fa1, SPARE
    la a0, WHOLE
    cy_gun_can_fire
    la t0, WHOLE
    lw t1, 0(t0)
    li t2, 1
    bne t1, t2, contact_done
    cy_gun_fire
    fmv.d fa0, fs5
    cy_gun_power
contact_done:
    j dodge

no_contact:
    # ---- none in the scan: look again where one was last seen ----
    la t0, last_slot
    lw a0, 0(t0)
    beqz a0, no_memory
    la a1, WHOLE
    cy_tracker_last_seen
    bnez a0, no_memory
    la t0, WHOLE
    lw t1, 0(t0)
    sub t1, s0, t1                  # ticks since it was seen
    li t2, 60
    bge t1, t2, no_memory
    la t0, last_slot
    lw a0, 0(t0)
    la a1, LAST
    cy_tracker_position
    bnez a0, no_memory
    la a0, ME
    la a1, LAST
    call bearing_to
    fmv.d fs3, fa0
    fcvt.d.w fs4, zero
    la t0, free
    lw t1, 0(t0)
    beqz t1, 1f
    la a0, ME
    la a1, LAST
    call elevation_to
    fmv.d fs4, fa0
1:  fload fa0, CLOSE_IN
    fmv.d fa1, fs3
    fmv.d fa2, fs4
    fload fa3, GAIN
    fmv.d fa4, fs0
    fmv.d fa5, fs1
    call steer
    # Sweep a narrow scan across that bearing: 8 degrees a step, -2..2.
    li t1, 5
    rem t1, s0, t1
    addi t1, t1, -2
    fcvt.d.w ft0, t1
    fload ft1, LOOK_STEP
    fmadd.d fa0, ft0, ft1, fs3
    fload fa1, LOOK_ARC
    fmv.d fa2, fs4
    call scan
    j dodge

no_memory:
    # ---- nothing remembered: follow a passive cue ----
    la a0, CUE
    cy_tracker_cue
    bnez a0, search
    la a0, ME
    la a1, CUE + CY_CUE_POSITION    # the cue's position, a vec3 inside it
    call bearing_to
    fmv.d fs3, fa0
    fcvt.d.w fs4, zero
    la t0, free
    lw t1, 0(t0)
    beqz t1, 1f
    la a0, ME
    la a1, CUE + CY_CUE_POSITION
    call elevation_to
    fmv.d fs4, fa0
1:  fload fa0, CUE_SPEED
    fmv.d fa1, fs3
    fmv.d fa2, fs4
    fload fa3, GAIN
    fmv.d fa4, fs0
    fmv.d fa5, fs1
    call steer
    # Sweep the scan across the cue's error, at most 30 degrees.
    la t0, CUE
    fld ft2, CY_CUE_BEARING_ERROR(t0)
    fload ft3, LOOK_ARC
    fmin.d ft2, ft2, ft3
    li t1, 5
    rem t1, s0, t1
    addi t1, t1, -2
    fcvt.d.w ft0, t1
    fload ft1, TWO
    fdiv.d ft0, ft0, ft1
    fmadd.d fa0, ft0, ft2, fs3
    fload fa1, LOOK_ARC
    fmv.d fa2, fs4
    call scan
    j dodge

search:
    # ---- nothing at all: wander and sweep the circle in half a second ----
    fload fa0, WANDER
    fmv.d fa1, fs0
    fcvt.d.w fa2, zero
    fload fa3, GAIN
    fmv.d fa4, fs0
    fmv.d fa5, fs1
    call steer
    fload fa0, SEARCH_TURN
    cy_drive_turn
    la a0, SCAN
    cy_scanner_last
    la t0, SCAN
    fld fa0, CY_SCAN_HEADING(t0)
    fload ft0, SWEEP_STEP
    fadd.d fa0, fa0, ft0
    fload fa1, SEARCH_ARC
    fcvt.d.w fa2, zero
    la t0, free
    lw t1, 0(t0)
    beqz t1, 1f
    li t1, 10                       # flying free: three heights in turn
    div t1, s0, t1
    li t2, 3
    rem t1, t1, t2
    addi t1, t1, -1
    fcvt.d.w ft0, t1
    fload ft1, SWEEP_PITCH
    fmul.d fa2, ft0, ft1
1:  call scan

dodge:
    # ---- a simple dodge: vary it, or predict the shooter ----
    # A muzzle cue means a robot fired: for half a second swerve off the
    # line of fire, left and right in turn. More cues than the array holds
    # (CY_SHORT) count as none heard.
    la t0, dodge_until
    lw t1, 0(t0)
    blt s0, t1, swerve
    la a0, CUES
    li a1, 8
    la a2, WHOLE
    cy_sensors_cues
    bnez a0, swerve
    la t0, WHOLE
    lw t1, 0(t0)                    # how many cues
    la t2, CUES
    li t3, CY_CONTACT_SIZE
1:  beqz t1, swerve
    lw t4, CY_CONTACT_SOURCE(t2)
    addi t4, t4, -CY_SOURCE_MUZZLE
    beqz t4, 2f
    add t2, t2, t3                  # the next contact
    addi t1, t1, -1
    j 1b
2:  addi t1, s0, 10                 # dodge for 10 ticks
    la t0, dodge_until
    sw t1, 0(t0)
    la t0, dodges
    lw t1, 0(t0)
    addi t1, t1, 1
    sw t1, 0(t0)

swerve:
    la t0, dodge_until
    lw t1, 0(t0)
    bge s0, t1, battery
    # Left on odd dodges: turn is positive left, strafe positive right.
    fload fs2, ONE
    la t0, dodges
    lw t1, 0(t0)
    andi t1, t1, 1
    bnez t1, 1f
    fneg.d fs2, fs2
1:  la t0, forces
    lw t1, 0(t0)
    beqz t1, 2f
    fload fa0, HALF
    cy_drive_throttle
    fneg.d fa0, fs2
    cy_drive_strafe
    j battery
2:  fload fa0, ONE
    cy_drive_throttle
    fmv.d fa0, fs2
    cy_drive_turn

battery:
    # ---- low battery: drive gently so the shots can still be paid for ----
    la a0, NUMBER
    cy_battery_charge
    la t0, NUMBER
    fld ft0, 0(t0)
    la a0, NUMBER
    cy_battery_capacity
    fld ft1, 0(t0)
    fload ft2, HALF
    fmul.d ft1, ft1, ft2
    fmul.d ft1, ft1, ft2            # a quarter of the capacity
    flt.d t1, ft0, ft1
    beqz t1, storm
    la a0, NUMBER
    cy_drive_get_throttle
    fld fa0, 0(t0)
    fmul.d fa0, fa0, ft2
    cy_drive_throttle

storm:
    # ---- the storm drifts across the arena and hurts anything inside it ----
    # Near its edge, step out across its path: at right angles to its
    # drift, on the side this robot is already on, so it passes by.
    la a0, STORM
    cy_nav_storm
    bnez a0, zone
    la a0, ME
    la a1, NUMBER
    cy_nav_storm_margin
    bnez a0, zone
    la t0, NUMBER
    fld ft0, 0(t0)
    fload ft1, STORM_EDGE
    flt.d t1, ft1, ft0
    beqz t1, zone
    la a0, STORM + CY_STORM_CENTER
    la a1, ME
    call bearing_to
    fmv.d fs2, fa0                  # away from its centre
    la a0, STILL
    la a1, STORM + CY_STORM_VELOCITY
    call bearing_to
    fmv.d fs3, fa0                  # the way it drifts
    fmv.d fa0, fs2
    fmv.d fa1, fs3
    call angle_diff
    fload ft0, QUARTER_TURN
    fcvt.d.w ft1, zero
    flt.d t1, fa0, ft1
    beqz t1, 1f
    fneg.d ft0, ft0
1:  fadd.d fa1, fs3, ft0
    fload fa0, ONE
    fcvt.d.w fa2, zero
    fload fa3, TIGHT_GAIN
    fmv.d fa4, fs0
    fmv.d fa5, fs1
    call steer

zone:
    # ---- sudden death: head back inside the safe zone ----
    la a0, ZONE
    cy_nav_zone
    bnez a0, done
    la a0, ME
    la a1, NUMBER
    cy_nav_zone_margin
    bnez a0, done
    la t0, NUMBER
    fld ft0, 0(t0)
    fload ft1, ZONE_EDGE
    flt.d t1, ft0, ft1
    beqz t1, done
    fcvt.d.w fs4, zero
    la t0, ZONE
    lw t1, CY_ZONE_SHAPE(t0)
    addi t1, t1, -CY_SHAPE_SPHERE
    bnez t1, 1f
    la a0, ME
    la a1, ZONE + CY_ZONE_CENTER
    call elevation_to
    fmv.d fs4, fa0
1:  la a0, ME
    la a1, ZONE + CY_ZONE_CENTER
    call bearing_to
    fmv.d fa1, fa0
    fload fa0, HOME_SPEED
    fmv.d fa2, fs4
    fload fa3, TIGHT_GAIN
    fmv.d fa4, fs0
    fmv.d fa5, fs1
    call steer

done:
    fld fs0, 0(sp)
    fld fs1, 8(sp)
    fld fs2, 16(sp)
    fld fs3, 24(sp)
    fld fs4, 32(sp)
    fld fs5, 40(sp)
    fld fs6, 48(sp)
    lw s1, 68(sp)
    lw s0, 72(sp)
    lw ra, 76(sp)
    addi sp, sp, 80
    li a0, 0
    ret
