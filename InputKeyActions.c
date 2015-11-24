/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                              */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/	 

// ---------------------------------------------------------------------------------------------------------
//	Functions:
//		- Direct trigger key and matrix key falling, rising, long pressing handling functions.
//			* Direct trigger key handling functions are referenced in a lookup table "g_asTriggerKeyHandler" (in "ConfigIO.h").
//			* Matrix key handling functions are reference in a lookup talbe "g_asMatrixKeyHandler" (in "ConfigIO.h").
//			* Both lookup tables are used by keypad library to call at key action being identified.
//		- Default trigger key and matrix key handler is "Default_KeyHandler()"
//
//	Reference "Readme.txt" for more information.
// ---------------------------------------------------------------------------------------------------------

#include "App.h"
#include "Keypad.h"
#include "./OLED/LY096BG30.h"
#include "config.h"

extern volatile UINT8 g_u8AppCtrl;

extern S_APP g_sApp;

extern BOOL App_StartPlay(void);
extern BOOL App_StopPlay(void);
extern void App_PowerDown(void);

void Playback_KeypadHandler(UINT32 u32Param)
{		
	if ( (g_u8AppCtrl&APPCTRL_PLAY) == 0 )
	{
		if( g_sApp.u32TotalAudioNum > 0 )
		{
			App_StartPlay();
			#ifdef OLED_ENABLE
			print_Line(1, "   PLAY  Start  ");	
			#endif
		}
	}
	else
	{
		App_StopPlay();
		#ifdef OLED_ENABLE
		print_Line(1, "   PLAY  Stop  ");	
		#endif
	}
}

void PlayNext_KeypadHandler(UINT32 u32Param)
{
	if ( g_u8AppCtrl&APPCTRL_PLAY )	
		Playback_StopPlay();	

	if( g_sApp.u32TotalAudioNum > 0 )
	{
		if( (g_sApp.u32PlayID+=1) >= g_sApp.u32TotalAudioNum )
			g_sApp.u32PlayID = 0;
		
		App_StartPlay();
		#ifdef OLED_ENABLE
		print_Line(1, "   PLAY  NEXT  ");
		#endif

	}
}

void PlayPrev_KeypadHandler(UINT32 u32Param)
{ 
	if ( g_u8AppCtrl&APPCTRL_PLAY )	 
		Playback_StopPlay();

	if( g_sApp.u32TotalAudioNum > 0 )
	{
		if( g_sApp.u32PlayID == 0 )
			g_sApp.u32PlayID = g_sApp.u32TotalAudioNum-1;
		else
			g_sApp.u32PlayID--;

		App_StartPlay();
		#ifdef OLED_ENABLE
		print_Line(1, "   PLAY  Prev  ");
		#endif

	}
}

void PowerDown_KeypadHandler(UINT32 u32Param)
{
	App_PowerDown();
}

void PlayPause_KeypadHandler(UINT32 u32Param)
{
	if ((g_u8AppCtrl&APPCTRL_PAUSE) == 0)
	{
		Playback_PauseCtrl(0, TRUE);
		g_u8AppCtrl |= APPCTRL_PAUSE;
	}
	else
	{
		Playback_PauseCtrl(0, FALSE);
		g_u8AppCtrl &= ~APPCTRL_PAUSE;
	}
}

void PlayMute_KeypadHandler(UINT32 u32Param)
{
	if ((g_u8AppCtrl&APPCTRL_MUTE) == 0)
	{
		Playback_MuteCtrl(0, TRUE);
		g_u8AppCtrl |= APPCTRL_MUTE;
	}
	else
	{
		Playback_MuteCtrl(0, FALSE);
		g_u8AppCtrl &= ~APPCTRL_MUTE;
	}
}

void Default_KeyHandler(UINT32 u32Param)
{
	// Within this function, key state can be checked with these keywords:
	//	TGnF: direct trigger key n is in falling state(pressing)
	//	TGnR: direct trigger key n is in rising state(releasing)
	//	TGnP: direct trigger key n is in long pressing state
	//	KEYmF: matrix key m is in falling state(pressing)
	//	KEYmR: matrix key m is in rising state(releasing)
	//	KEYmP: matrix key m is in long pressing state
	// the maxium value of n is defined by "TRIGGER_KEY_COUNT" in "ConfigIO.h"
	// the maxium value of m is defined by "MATRIX_KEY_COUNT"  in "ConfigIO.h"
	switch(u32Param)
	{
	#if 0
		case TG1F:
			Playback_KeypadHandler(0);
			break;
		case TG2F:
			PlayNext_KeypadHandler(0);
			break;
		case TG3F:
			PlayPrev_KeypadHandler(0);
			break;	
		case TG4P:
			PowerDown_KeypadHandler(0);	
			break;	
		case TG5F:
			PlayMute_KeypadHandler(0);
			break;	
		case TG6F:
			PlayPause_KeypadHandler(0);
			break;	
	#endif
		case TG3F:
			#ifdef DEBUG_ENABLE
			printf ("PA7(SWA7) is falling state.\n"); 
			#endif
			PlayPrev_KeypadHandler(0);
			break;
		case TG2F:
			#ifdef DEBUG_ENABLE
			printf ("PA6(SWA6) is falling state.\n"); 
			#endif
			Playback_KeypadHandler(0);
			break;
		case TG1F:
			#ifdef DEBUG_ENABLE
			printf ("PA5(SWA5) is falling state.\n"); 
			#endif
			PlayNext_KeypadHandler(0);
			break;
		default:
			break;

	}
}
