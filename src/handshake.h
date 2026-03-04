#ifndef HANDSHAKE_H
#define HANDSHAKE_H

void handshakeTask(void * pvParameters);

extern std::atomic<uint8_t> device_num;

#endif