# Microsoft Developer Studio Project File - Name="Orbiter Projects" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Dynamic-Link Library" 0x0102

CFG=Orbiter Projects - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "Orbiter Projects.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "Orbiter Projects.mak" CFG="Orbiter Projects - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "Orbiter Projects - Win32 Release" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE "Orbiter Projects - Win32 Debug" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "Orbiter Projects - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MT /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "ORBITERPROJECTS_EXPORTS" /YX /FD /c
# ADD CPP /nologo /MT /W3 /GX /O2 /I "..\..\include" /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "ORBITERPROJECTS_EXPORTS" /YX /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /dll /machine:I386
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib opengl32.lib glu32.lib orbiter.lib orbitersdk.lib /nologo /dll /machine:I386 /out:"..\..\..\Modules\Dragonfly.dll" /libpath:"..\..\lib"
# SUBTRACT LINK32 /incremental:yes

!ELSEIF  "$(CFG)" == "Orbiter Projects - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MTd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "ORBITERPROJECTS_EXPORTS" /YX /FD /GZ /c
# ADD CPP /nologo /MTd /W3 /Gm /GX /ZI /Od /I "..\..\include" /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "ORBITERPROJECTS_EXPORTS" /YX /FD /GZ /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /dll /debug /machine:I386 /pdbtype:sept
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib opengl32.lib glu32.lib orbiter.lib orbitersdk.lib /nologo /dll /debug /machine:I386 /out:"..\..\..\Modules\Dragonfly.dll" /pdbtype:sept /libpath:"..\..\lib"

!ENDIF 

# Begin Target

# Name "Orbiter Projects - Win32 Release"
# Name "Orbiter Projects - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Group "Panel"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\instruments.cpp
# End Source File
# Begin Source File

SOURCE=.\panel.cpp
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\resource.h
# End Source File
# End Group
# Begin Group "Electrical"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\esystems.cpp
# End Source File
# End Group
# Begin Group "Hydraulic"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\hsystems.cpp
# End Source File
# Begin Source File

SOURCE=.\thermal.cpp
# End Source File
# End Group
# Begin Group "Math"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\matrix.cpp
# End Source File
# Begin Source File

SOURCE=.\quaternion.cpp
# End Source File
# Begin Source File

SOURCE=.\vectors.cpp
# End Source File
# End Group
# Begin Source File

SOURCE=.\Dragonfly.cpp
# End Source File
# Begin Source File

SOURCE=.\internal.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\Dragonfly.h
# End Source File
# Begin Source File

SOURCE=.\esystems.h
# End Source File
# Begin Source File

SOURCE=.\hsystems.h
# End Source File
# Begin Source File

SOURCE=.\instruments.h
# End Source File
# Begin Source File

SOURCE=.\internal.h
# End Source File
# Begin Source File

SOURCE=.\matrix.h
# End Source File
# Begin Source File

SOURCE=.\panel.h
# End Source File
# Begin Source File

SOURCE=.\quaternion.h
# End Source File
# Begin Source File

SOURCE=.\thermal.h
# End Source File
# Begin Source File

SOURCE=.\vectors.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# Begin Source File

SOURCE=.\ADIborder.bmp
# End Source File
# Begin Source File

SOURCE=.\cb.bmp
# End Source File
# Begin Source File

SOURCE=.\cw.bmp
# End Source File
# Begin Source File

SOURCE=.\Dial1.bmp
# End Source File
# Begin Source File

SOURCE=.\dock.bmp
# End Source File
# Begin Source File

SOURCE=.\dragon.bmp
# End Source File
# Begin Source File

SOURCE=.\egauge.bmp
# End Source File
# Begin Source File

SOURCE=.\Fonts.bmp
# End Source File
# Begin Source File

SOURCE=.\front.bmp
# End Source File
# Begin Source File

SOURCE=.\fuel.bmp
# End Source File
# Begin Source File

SOURCE=.\gauge.bmp
# End Source File
# Begin Source File

SOURCE=.\grill.bmp
# End Source File
# Begin Source File

SOURCE=.\left.bmp
# End Source File
# Begin Source File

SOURCE=.\MFD.bmp
# End Source File
# Begin Source File

SOURCE=.\nav.bmp
# End Source File
# Begin Source File

SOURCE=.\panel.rc
# End Source File
# Begin Source File

SOURCE=.\rad2.bmp
# End Source File
# Begin Source File

SOURCE=.\radar.bmp
# End Source File
# Begin Source File

SOURCE=.\radar2.bmp
# End Source File
# Begin Source File

SOURCE=.\right.bmp
# End Source File
# Begin Source File

SOURCE=.\rotary.bmp
# End Source File
# Begin Source File

SOURCE=.\SL.bmp
# End Source File
# Begin Source File

SOURCE=.\switch.bmp
# End Source File
# Begin Source File

SOURCE=.\Switch1.bmp
# End Source File
# Begin Source File

SOURCE=.\Switch2.bmp
# End Source File
# Begin Source File

SOURCE=.\tb.bmp
# End Source File
# Begin Source File

SOURCE=.\Up.bmp
# End Source File
# Begin Source File

SOURCE=.\vrot.bmp
# End Source File
# Begin Source File

SOURCE=.\vrotbk.bmp
# End Source File
# Begin Source File

SOURCE=.\warn.bmp
# End Source File
# End Group
# End Target
# End Project
