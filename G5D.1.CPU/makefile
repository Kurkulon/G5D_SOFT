
##################################################################################################

!ifndef version
version = Release
!endif

##################################################################################################

curdir = $+ $(%cdrive):$(%cwd) $-
cppdir = cpp
!ifneq version Win32_Debug
objdir = $(targetcpu)_$(version)
!else
objdir = $(version)
!endif

##################################################################################################

.BEFORE
	@if NOT EXIST $(objdir) @md $(objdir) >nul

##################################################################################################

make : .SYMBOLIC
	@call wmake -h -f mkmodules cppdir=$(cppdir)\ objdir=$(objdir)\
!ifneq version Win32_Debug
	@call wmake -h -f makedep targetcpu=$(targetcpu) version=$(version) cppdir=$(cppdir) objdir=$(objdir)
!endif
	@call wmake $(__MAKEOPTS__) -f makeobj targetcpu=$(targetcpu) version=$(version) cppdir=$(cppdir) objdir=$(objdir)


##################################################################################################

