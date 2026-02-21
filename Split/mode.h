#pragma once 

uint8_t status;

unsigned long beforeFix;

void standbyMode(float h);
void ascending(float h);
void descending(float h);
void waitForFix(unsigned long now);
void rtwpMode();
void land(float h);

void modeRun(float h, unsigned long now);
