@echo off
setlocal enabledelayedexpansion

set LIST_CPP=modules_cpp = 
set LIST_OBJ=modules_obj = 

for %%i in ("%1*.cpp") do (
	set TEMP=%%i 
	set LIST_CPP=!LIST_CPP!!TEMP!
	set TEMP=!TEMP:.cpp=.o!
	set TEMP=!TEMP:%1=%2!
	set LIST_OBJ=!LIST_OBJ!!TEMP!
	rem echo %%i
	rem echo !TEMP!
)

echo !LIST_CPP! > %2\mkoutcpp
echo !LIST_OBJ! > %2\mkoutobj

