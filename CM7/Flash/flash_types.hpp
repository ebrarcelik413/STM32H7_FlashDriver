#pragma once

#include <cstdint>

enum class FlashStatus : uint8_t
{
    Ok = 0,

    InvalidArgument,
    InvalidAddress,
    InvalidLength,
    AlignmentError,

    NotInitialized,
    Busy,
    Timeout,

    UnlockError,
    LockError,
    ProgramError,
    EraseError,
    VerifyError,
	NotErased,

    NoSpace
};
