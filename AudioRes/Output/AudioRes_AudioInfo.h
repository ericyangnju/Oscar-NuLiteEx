#ifndef _AUDIOINFO_H_
#define _AUDIOINFO_H_

enum Audio_ID_Table
{
	SAMPLEFILE=0,	//NUL
	SOUND_MODLITWADZIEWICY_8K=1,	//NUL
	SPEECH_JAMES_8K=2,	//NUL
	SOUND_CANON_8K=3,	//NUL
};
#define AUDIOSYN_SOUND_MAX_ID	3

enum Audio_Equ_ID_Table
{
	EQU1=0,
};
// Define AudioRes_AudioInfoMerge.ROM size (without MIDI WavTable)
#define AUDIOINFO_ROM_NO_WTB_SIZE	99128
// Define MIDI WavTable size
#define MIDISYN_WTB_SIZE	0
// Define AudioRes_AudioInfoMerge.ROM size (with MIDI WavTable)
#define AUDIOINFO_ROM_SIZE	99128

#endif

