#pragma once

#define LIDAR_BUFF_LEN 860

void InitHelper();

float GetFixX();
float GetFixY();
float GetFaceX();
float GetFaceY();
void SetRanges(float* inData);
void SetTarget(int inX, int inY, bool inForce);
int GetHelperNum();
bool OutLine();
bool ChkTarget(int inX, int inY);
void ClearObst();
void ClearTarget();
void SetBorder(float inBorder);
int ResetNum(int inNum);