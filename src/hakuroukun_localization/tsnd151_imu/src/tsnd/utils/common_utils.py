def check_range(val_name, val, min_, max_):
    if val < min_ or max_ < val:
        raise ValueError("Invalid '{}': range is [{},{}] but {}".format(val_name, min_, max_, val))


def resample_df(df, ix_to):
    """Resampling a df with the new ix

    Extended description of function.

    Parameters
    ----------
    df: pandas.DataFrame
        pandas.DataFrame
    ix_to: list, tuple
        requirement: type(ix_to) == type(df.index)

    Returns
    -------
    df
        resampled pandas.DataFrame
    """
    import pandas as pd
    import numpy as np
    new_ix = ix_to[np.invert(np.in1d(ix_to, df.index))]  # remove duplicated index
    tmp_df = df.append(pd.DataFrame(np.repeat(np.NaN, len(new_ix)), index=new_ix))
    return tmp_df.sort_index().interpolate('index').loc[ix_to]


def set_jp_locale():
    import platform
    import locale
    jp_locale_str = 'ja_JP.UTF-8'
    if platform.system() == 'Windows':
        jp_locale_str = 'Japanese_Japan.932'

    locale.setlocale(locale.LC_TIME, f'{jp_locale_str}')

