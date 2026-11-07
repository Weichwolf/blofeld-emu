PulseAudio callback type correction

pulseaudio-callback-types.patch preserves PulseAudio 17.0 callback behavior
while using function signatures accepted by LLVM 19 and UBSan. It applies only
to the hash-pinned extracted build copy. Upstream notices remain in that copy.
