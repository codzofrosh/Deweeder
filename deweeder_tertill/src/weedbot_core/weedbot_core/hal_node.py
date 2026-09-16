"""Compatibility entry point using the same protected HAL implementation."""
from weedbot_core.hal_hw_gpio import HalHW as HalNode, main

if __name__ == '__main__':
    main()
