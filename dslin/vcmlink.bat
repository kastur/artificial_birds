@if not "%1" == ""  goto START
@ECHO    /****************************************************************/
@ECHO    /**                       C L I N K                            **/
@ECHO    /**                                                            **/
@ECHO    /** CLINK links programs using the DISLIN C-library.           **/
@ECHO    /**                                                            **/
@ECHO    /** Command:  CLINK  [option] [-r8] [-shared]   main           **/
@ECHO    /**                                                            **/
@ECHO    /** option    is an optional qualifier that may have one of    **/
@ECHO    /**           the following values:                            **/
@ECHO    /**      -c   to compile programs before linking               **/
@ECHO    /**      -cpp to compile C++ programs before linking           **/
@ECHO    /**      -r   to run programs after linking                    **/
@ECHO    /**      -a   to compile, link and run programs.               **/
@ECHO    /**                                                            **/
@ECHO    /** -r8       is an optional parameter for using the double    **/
@ECHO    /**           precision library of DISLIN.                     **/
@ECHO    /** -shared   is an optional parameter for using the shareable **/
@ECHO    /**           library of DISLIN.                               **/
@ECHO    /**                                                            **/
@ECHO    /** main      is the name of the main program.                 **/
@ECHO    /**                                                            **/
@ECHO    /** Example:  CLINK  -a   TEST                                 **/
@ECHO    /** Version:  MS-Visual C++ 8.x                                **/
@ECHO    /****************************************************************/
@goto EXIT
:START
@set _dislin=%DISLIN%
@if "%DISLIN%" == "" set _dislin=c:\dislin
@set _opt1=%1
@if %1 ==  -c	shift
@if %1 ==  -cpp	shift
@if %1 ==  -a	shift
@if %1 ==  -r	shift
@set _opt2=%1
@if %1 ==  -r8	shift
@set _opt3=%1
@if %1 ==  -shared  shift
@rem
@if %_opt1% ==  -c    goto COMP
@if %_opt1% ==  -cpp  goto COMP
@if %_opt1% ==  -a    goto COMP
@goto  LINK
:COMP
@set _ext=c
@set _inc=%_dislin%
@if %_opt1% ==  -cpp  set _ext=cpp
@if %_opt2% == -r8 set _inc=%_dislin%\real64
cl /c /I%_inc%  %1.%_ext%
@set _ext=
@set _inc=
@if errorlevel 1 goto ENDE
:LINK
@set _lib=disvcm.lib
@if %_opt2% == -r8      set _lib=disvcm_d.lib
@if not %_opt3% == -shared  goto LINK2
@set _lib=dislnc.lib
@if %_opt2% == -r8      set _lib=dislnc_d.lib
:LINK2
link %1 %2 %3 %4 %5 %6  %_dislin%\%_lib% /DEFAULTLIB:gdi32.lib user32.lib
@set _lib=
@if %_opt1% ==  -a  goto RUN
@if %_opt1% ==  -r  goto RUN
@goto ENDE
:RUN
@if errorlevel 1 goto ENDE
%1
:ENDE
@set _dislin=
@set _opt1=
@set _opt2=
@set _opt3=
:EXIT
