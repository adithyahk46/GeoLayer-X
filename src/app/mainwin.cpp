/***************************************************************************
    mainwin.cpp
    ---------------------
    begin                : February 2017
    copyright            : (C) 2017 by Juergen E. Fischer
    email                : jef at norbit dot de
 ***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include <fstream>
#include <filesystem>
#include <io.h>
#include <iostream>
#include <list>
#include <memory>
#include <sstream>
#include <windows.h>
#include <string>
#include <vector>
#include <cstdio>

void showError( std::string message, std::string title )
{
  std::string newmessage = "Oops, looks like an error loading GeoLayer-X \n\n Details: \n\n" + message;
  MessageBox( NULL, newmessage.c_str(), title.c_str(), MB_ICONERROR | MB_OK );
  std::cerr << message << std::endl;
}

std::string moduleExeBaseName( void )
{
  DWORD l = MAX_PATH;
  std::unique_ptr<char> filepath;
  for ( ;; )
  {
    filepath.reset( new char[l] );
    if ( GetModuleFileName( nullptr, filepath.get(), l ) < l )
      break;

    l += MAX_PATH;
  }

  std::string basename( filepath.get() );
  return basename;
}

int CALLBACK WinMain( HINSTANCE /*hInstance*/, HINSTANCE /*hPrevInstance*/, LPSTR /*lpCmdLine*/, int /*nCmdShow*/ )
{
  bool debugMode = false;
  std::vector<char *> applicationArgs;
  for ( int i = 0; i < __argc; ++i )
  {
    if ( std::string( __argv[i] ) == "--debug" )
      debugMode = true;
    else
      applicationArgs.push_back( __argv[i] );
  }
  applicationArgs.push_back( nullptr );

  if ( debugMode )
  {
    if ( !AttachConsole( ATTACH_PARENT_PROCESS ) && GetLastError() != ERROR_ACCESS_DENIED )
    {
      showError( "Could not attach to the parent console for --debug.", "Error loading GeoLayer-X" );
      return EXIT_FAILURE;
    }

    if ( !std::freopen( "CONOUT$", "w", stdout ) || !std::freopen( "CONOUT$", "w", stderr ) )
    {
      showError( "Could not redirect debug output to the parent console.", "Error loading GeoLayer-X" );
      return EXIT_FAILURE;
    }

    if ( _putenv( "QT_FORCE_STDERR_LOGGING=1" ) < 0 )
    {
      showError( "Could not enable Qt debug output in the parent console.", "Error loading GeoLayer-X" );
      return EXIT_FAILURE;
    }
  }

  std::string exename( moduleExeBaseName() );
  std::string basename( exename.substr( 0, exename.size() - 4 ) );
  std::filesystem::path executablePath( exename );
  std::filesystem::path envFile = executablePath.parent_path() / "glx.env";
  std::string glxRoot = executablePath.parent_path().parent_path().lexically_normal().string();

  try
  {
    std::string rootVariable = "GLX_ROOT=" + glxRoot;
    if ( _putenv( rootVariable.c_str() ) < 0 )
    {
      showError( "Could not set environment variable GLX_ROOT", "Error loading GeoLayer-X" );
      return EXIT_FAILURE;
    }

    std::ifstream file;
    file.open( envFile );
    if ( !file.is_open() )
    {
      std::string message = "Could not read environment file " + envFile.string();
      showError( message, "Error loading GeoLayer-X" );
      return EXIT_FAILURE;
    }

    std::string var;
    while ( std::getline( file, var ) )
    {
      size_t separator = var.find( '=' );
      if ( separator != std::string::npos && var.substr( 0, separator ) == "GLX_ROOT" )
        continue;

      std::string value = separator == std::string::npos ? var : var.substr( separator + 1 );
      DWORD expandedSize = ExpandEnvironmentStringsA( value.c_str(), nullptr, 0 );
      if ( expandedSize == 0 )
      {
        std::string message = "Could not expand environment variable: " + var;
        showError( message, "Error loading GeoLayer-X" );
        return EXIT_FAILURE;
      }

      std::vector<char> expandedValue( expandedSize );
      DWORD expandedLength = ExpandEnvironmentStringsA( value.c_str(), expandedValue.data(), expandedSize );
      if ( expandedLength == 0 || expandedLength > expandedSize )
      {
        std::string message = "Could not expand environment variable: " + var;
        showError( message, "Error loading GeoLayer-X" );
        return EXIT_FAILURE;
      }

      std::string expandedVar = separator == std::string::npos
        ? std::string( expandedValue.data() )
        : var.substr( 0, separator + 1 ) + expandedValue.data();
      if ( _putenv( expandedVar.c_str() ) < 0 )
      {
        std::string message = "Could not set environment variable:" + expandedVar;
        showError( message, "Error loading GeoLayer-X" );
        return EXIT_FAILURE;
      }
    }
  }
  catch ( std::ifstream::failure &e )
  {
    std::string message = "Could not read environment file " + envFile.string() + " [" + e.what() + "]";
    showError( message, "Error loading GeoLayer-X" );
    return EXIT_FAILURE;
  }

#ifndef _MSC_VER // MinGW
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif
  HINSTANCE hKernelDLL = LoadLibrary( "kernel32.dll" );
  BOOL ( *SetDefaultDllDirectories )( DWORD ) = hKernelDLL ? reinterpret_cast<BOOL ( * )( DWORD )>( GetProcAddress( hKernelDLL, "SetDefaultDllDirectories" ) ) : 0;
  DLL_DIRECTORY_COOKIE ( *AddDllDirectory )( PCWSTR ) = hKernelDLL ? reinterpret_cast<DLL_DIRECTORY_COOKIE ( * )( PCWSTR )>( GetProcAddress( hKernelDLL, "AddDllDirectory" ) ) : 0;
#ifndef _MSC_VER // MinGW
#pragma GCC diagnostic pop
#endif

  if ( SetDefaultDllDirectories && AddDllDirectory )
  {
    SetDefaultDllDirectories( LOAD_LIBRARY_SEARCH_DEFAULT_DIRS );

    wchar_t windir[MAX_PATH];
    GetWindowsDirectoryW( windir, MAX_PATH );
    wchar_t systemdir[MAX_PATH];
    GetSystemDirectoryW( systemdir, MAX_PATH );

    wchar_t *path = wcsdup( _wgetenv( L"PATH" ) );

#ifdef _UCRT
    for ( wchar_t *p = wcstok( path, L";", nullptr ); p; p = wcstok( NULL, L";", nullptr ) )
#else
    for ( wchar_t *p = wcstok( path, L";" ); p; p = wcstok( NULL, L";" ) )
#endif
    {
      if ( wcsicmp( p, windir ) == 0 )
        continue;
      if ( wcsicmp( p, systemdir ) == 0 )
        continue;
      AddDllDirectory( p );
    }

    free( path );
  }


#ifdef _MSC_VER
  HINSTANCE hGetProcIDDLL = LoadLibrary( "glx_app.dll" );
#else
  // MinGW
  HINSTANCE hGetProcIDDLL = LoadLibrary( "libglx_app.dll" );
#endif

  if ( !hGetProcIDDLL )
  {
    DWORD error = GetLastError();
    LPTSTR errorText = NULL;

    FormatMessage( FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, error, MAKELANGID( LANG_NEUTRAL, SUBLANG_DEFAULT ), ( LPTSTR ) &errorText, 0, NULL );

    std::string message = "Could not load glx_app.dll \n Windows Error: " + std::string( errorText ) + "\n Help: \n\n Check " + basename + ".env for correct environment paths";
    showError( message, "Error loading GeoLayer-X" );

    LocalFree( errorText );
    errorText = NULL;
    return EXIT_FAILURE;
  }

#ifndef _MSC_VER // MinGW
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif
  int ( *realmain )( int, char *[] ) = ( int ( * )( int, char *[] ) ) GetProcAddress( hGetProcIDDLL, "glx_main" );
#ifndef _MSC_VER // MinGW
#pragma GCC diagnostic pop
#endif

  if ( !realmain )
  {
    showError( "Could not locate glx_main function in glx_app.dll", "Error loading GeoLayer-X" );
    return EXIT_FAILURE;
  }

  return realmain( static_cast<int>( applicationArgs.size() - 1 ), applicationArgs.data() );
}
