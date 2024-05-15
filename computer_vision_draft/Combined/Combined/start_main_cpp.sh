#!/bin/bash

# Start main.cpp program


cd /home/radxa/E7012E/E7012E_software/hosted

chown radxa /dev/spidev1.0

su - radxa <<!
radxa
!

/home/radxa/.cargo/bin/cargo build --example tcp_spi --release

./target/release/examples/tcp_spi&

/home/radxa/E7012E/E7012E_software/computer_vision_draft/Combined/Combined/OpenCV 

echo "DONE :)"
