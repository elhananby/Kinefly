from kinefly.core.setdict import set_dict_with_overwrite, set_dict_with_preserve


def test_preserve_keeps_existing_keys():
    target = {"a": 1, "b": 2}
    source = {"a": 0, "c": 0}
    set_dict_with_preserve(target, source)
    assert target == {"a": 1, "b": 2, "c": 0}


def test_overwrite_replaces_existing_keys():
    target = {"a": 1, "b": 2}
    source = {"a": 0, "c": 0}
    set_dict_with_overwrite(target, source)
    assert target == {"a": 0, "b": 2, "c": 0}


def test_nested_dict_preserve():
    target = {"x": {"a": 1}}
    source = {"x": {"a": 99, "b": 2}, "y": {"c": 3}}
    set_dict_with_preserve(target, source)
    assert target == {"x": {"a": 1, "b": 2}, "y": {"c": 3}}


def test_nested_dict_overwrite():
    target = {"x": {"a": 1}}
    source = {"x": {"a": 99, "b": 2}}
    set_dict_with_overwrite(target, source)
    assert target == {"x": {"a": 99, "b": 2}}
