_shutdown_requested = False


def request_shutdown():
    global _shutdown_requested
    _shutdown_requested = True


def clear_shutdown():
    global _shutdown_requested
    _shutdown_requested = False


def is_shutdown_requested():
    return _shutdown_requested
