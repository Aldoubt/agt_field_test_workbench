"""Keep supervised field trials independent of formal acceptance paperwork."""


def require_navigation_calibration(mode, supervised, robot, selected):
    """Return whether a formal navigation report is required; never alter calibration."""
    if mode not in ('formal', 'field_test'):
        raise ValueError('navigation_mode must be formal or field_test')
    if not selected or selected.get('robot') != robot:
        raise ValueError('navigation requires a selected calibration for this robot')
    if mode == 'field_test':
        if robot != 'bunker' or not supervised:
            raise ValueError('field_test navigation currently requires supervised Bunker operation')
        if selected.get('status') not in ('CANDIDATE', 'VERIFIED'):
            raise ValueError('field_test requires a valid candidate or verified calibration')
        return False
    if selected.get('status') != 'VERIFIED':
        raise ValueError('VERIFIED mounting calibration required for formal navigation')
    return True
