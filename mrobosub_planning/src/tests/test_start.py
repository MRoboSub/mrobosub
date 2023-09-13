from common import Start

@pytest.fixture
def start():
    return Start()

def test_null(start):
    start.initialize(None)
    