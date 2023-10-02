def str_to_int(string: str) -> int:
    string = string.strip().lower()
    words = string.split(' ') if ' ' in string else [[ch] for ch in string]
    return int(str(ord(words[0][0])) + str(ord(words[1][0])))


def msg_port_from_name(name: str):
    number = str_to_int(name)
    return number % 8192


def srv_port_from_name(name: str):
    number = str_to_int(name)
    return number % 512


def id_from_name(name: str):
    number = str_to_int(name)
    return number % 100
