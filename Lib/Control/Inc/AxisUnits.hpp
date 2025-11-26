#pragma once

// 単位ヘルパ（線形(mm/m) と 角度(rad) の換算を明示化）
// 備考: 現行システムでは、Encoder/gear_ratio により直線機構を擬似的にrad系へ写像
//（例: 一部軸の positionRad() は実質mmを返す）。ここでは既存スケーリングを変更せず、
// 明確化と将来のパラメータ化のためにヘルパのみ提供する。

static inline float mm_to_m(float mm) { return mm * 0.001f; }
static inline float m_to_mm(float m)  { return m * 1000.0f; }
static inline float mmps_to_mps(float mmps) { return mmps * 0.001f; }
static inline float mps_to_mmps(float mps)  { return mps * 1000.0f; }
