# A single diagnostic message attached to a frame (used in both FrameInfo and FrameError).

int32 code      # Numeric message code defined by the PhoXi API.
int32 severity  # Severity level: 0 = info, 1 = warning, 2 = error.
string text     # Human-readable message text.
