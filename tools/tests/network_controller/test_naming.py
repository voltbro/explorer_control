from network_controller.naming import port_from_name


def test_port_names_full_words():
    assert port_from_name('Angular Velocity') == 6586
    assert port_from_name('Self Diagnostic') == 176


def test_port_names_abbreviations():
    assert port_from_name('AV') == 6586
    assert port_from_name('SD') == 176
