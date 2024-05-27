#!/bin/bash
west build -p -b xiao_esp32s3
west flash
west espressif monitor