import traceback
from src.utils.logging_engine import logger
from algorithm.main import main


if __name__ == '__main__':
    try:
        main()
        print("SUCCESS")
    except Exception as e:
        logger.error("Failed to run algorithm")
        logger.error(f"Error: {e}, {traceback.format_exc()}")
        print("FAIL")
