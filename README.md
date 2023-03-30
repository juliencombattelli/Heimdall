# Heimdall

Heimdall, a pun for MDAL, stands for Modern Device Access Layer.
It is a vendor-independant library enabling a consistent and portable access to memory-mapped device registers.
Written using C++20, this library targets efficiency, safety, and simplicity of use.
Most errors can be detected at build time when the corresponding input data are known during the compilation (eg. invalid write access to a read-only device register, invalid value written into a bitfield, etc).
