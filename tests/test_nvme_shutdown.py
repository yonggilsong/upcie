import pytest


def test_nvme_shutdown(cijoe):
    """
    Run the check that a controller is notified of shutdown before it is disabled.

    The registers are host memory, so unlike the rest of the suite this needs no
    device and runs wherever uPCIe is deployed.
    """

    binary = "test_nvme_shutdown"

    err, _ = cijoe.run(f"which {binary}")
    if err:
        pytest.skip(f"{binary} is not installed on the target")

    err, _ = cijoe.run(binary)
    assert not err
