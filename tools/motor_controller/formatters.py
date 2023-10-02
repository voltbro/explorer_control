from voltbro.config.six_step.log_1_0 import log_1_0
from voltbro.config.six_step.get_1_0 import get_1_0
from voltbro.config.six_step.log_item_1_0 import log_item_1_0
from voltbro.config.six_step.values_1_0 import values_1_0


def f(val):
    s = f"{val.value:.2f}" if isinstance(val.value, float) else f"{val.value}"
    return s.rjust(5)

def line(s):
    return s.ljust(61)

def tl(s):
    return f"**{s}**".ljust(11)

def format_control_log(log: log_1_0.Response, get_config: get_1_0.Response):
    if log is None and get_config is None:
        return "None"
    view = ""

    if log is not None:
        l: log_item_1_0 = log.log_item
        view += "".join((line(s) for s in [
            f"{tl('CURRENT')}: {f(l.I_A)}, {f(l.I_B)}, {f(l.I_C)}",
            f"{tl('MOTOR')}: *theta* {f(l.elec_theta)}, *speed* {f(l.speed)}, *target* {f(l.target_speed)}",
            f"{tl('STATE')}: *is_stalling* {f(l.is_stalling)},  *PWM* {f(l.PWM)}",
            f"{tl('PIDs')}: *velocity signal* {f(l.velocity_report.signal)}, *current signal* {f(l.current_report.signal)}",
            "-"*52,
        ]))

    if get_config is not None:
        c: values_1_0 = get_config.config

        for attrn, dispn in (
            ('I_mult', 'I*'),
            ('PWM_mult', 'PWM*'),
            ('sampling_interval', 'sample T'),
            ('speed_const', 'Vc'),

            ('encoder_filtering', '**FILTERs**: encoder'),
            ('speed_filtering', 'speed'),

            (('velocity_pid', 'p_gain'), '**PIDs**: vel'),
            (('velocity_pid', 'i_gain'), ''),
            (('velocity_pid', 'd_gain'), ''),
            (('current_pid', 'p_gain'), 'cur'),
            (('current_pid', 'i_gain'), ''),
            (('current_pid', 'd_gain'), ''),

             ('stall_current', '**STALL** (I, T)'),
             ('stall_timeout', ''),
        ):
            if isinstance(attrn, str):
                attrn = (attrn,)
            obj = c
            for attrn_i in attrn:
                obj = getattr(obj, attrn_i)
            view += (f'{dispn} ' if dispn else '') + (f"{obj.value:.2f}" if isinstance(obj.value, float) else str(obj.value)) + ", "

    return view
