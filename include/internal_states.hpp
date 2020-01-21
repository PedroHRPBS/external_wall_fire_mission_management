#pragma once
#include <stdint.h>

enum class internal_state{NOT_READY=0, ERROR=1, READY_TO_START=2, SCANNING_OUTDOOR=3, APPROACHING_OUTDOOR=4, 
                          EXTINGUISHING_OUTDOOR=5, RETURNING_TO_BASE=6, FINISHED=7};