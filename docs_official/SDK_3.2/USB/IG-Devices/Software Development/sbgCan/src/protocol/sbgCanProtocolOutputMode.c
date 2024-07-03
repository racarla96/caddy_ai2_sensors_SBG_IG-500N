#include "sbgCanProtocolOutputMode.h"

//----------------------------------------------------------------------//
//- Internal swap functions                                            -//
//----------------------------------------------------------------------//

// Union used to store both float and integer in raw format (usefull to avoid types conversions)
typedef union _SwapUnion32
{
	float valFloat;
	int32 valInt;
} SwapUnion32;

// Union used to store both double and integer in raw format (usefull to avoid types conversions)
typedef union _SwapUnion64
{
	double valFloat;
	int64 valInt;
} SwapUnion64;

//
// Swap a 16 bits integer
//
uint16 swap16(uint16 x) 
{
	return((x<<8)|(x>>8));
}

//
// Swap a 32 bits integer
//
uint32 swap32(uint32 x) 
{
	return((x<<24)|((x<<8)&0x00FF0000)|((x>>8)&0x0000FF00)|(x>>24));
}

//
// Swap a 64 bits integer
//
uint64 swap64(uint64 x) 
{
	uint32 hi, lo;

	//
	// Separate into high and low 32-bit values
	//
	lo = (uint32)(x&0xFFFFFFFF);
	x >>= 32;
	hi = (uint32)(x&0xFFFFFFFF);

	//
	// Swap each part and rebuild the 64 bit vale
	//
	x = swap32(lo);
	x <<= 32;
	x |= swap32(hi);

	return x;
}

//----------------------------------------------------------------------//
//- Methods used to convert data from host to target format            -//
//----------------------------------------------------------------------//

/*!
 *	Convert a 16 bits integer from host to target endianness.
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
int16 sbgCanHostToTarget16(int16 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN
		
		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap16(val);

	#else
		#error	sbgCanHostToTarget16: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a 16 bits integer from host to target endianness. (unsigned)
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
uint16 sbgCanHostToTargetU16(uint16 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN
		
		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap16(val);

	#else
		#error	sbgCanHostToTarget16: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a 32 bits integer from host to target endianness.
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
int32 sbgCanHostToTarget32(int32 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN

		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap32(val);

	#else
		#error	sbgCanHostToTarget32: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a 32 bits integer from host to target endianness. (unsigned)
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
uint32 sbgCanHostToTargetU32(uint32 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN

		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap32(val);

	#else
		#error	sbgCanHostToTarget32: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a 64 bits integer from host to target endianness.
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
int64 sbgCanHostToTarget64(int64 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN

		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap64(val);

	#else
		#error	sbgCanHostToTarget64: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a 64 bits integer from host to target endianness. (unsigned)
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
uint64 sbgCanHostToTargetU64(uint64 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN

		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap64(val);

	#else
		#error	sbgCanHostToTarget64: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a 32 bits float from host endianness to a fixed32 in target endianness
 *	\param[in]	val						Value in host endianness in float.
 *	\return								Value in target endianness float.
 */
uint32 sbgCanHostToTargetFixed32(float val)
{
	SwapUnion32 rawVal;

	//
	// We have to convert the float into fixed point format
	//
	rawVal.valInt = (int32)(val*0x100000);

	//
	// Handle endianess issues
	//
	return sbgCanHostToTarget32(rawVal.valInt);
}

/*!
 *	Convert a 64 bits double from host endianness to a fixed64 in target endianness.
 *	\param[in]	val						Value in host endianness in double.
 *	\return								Value in target endianness in double.
 */
uint64 sbgCanHostToTargetFixed64(double val)
{
	SwapUnion64 rawVal;

	//
	// We have to convert the float into double point format
	//
	rawVal.valInt = (int64)(val*0x100000000LL);

	//
	// Handle endianess issues
	//
	return sbgCanHostToTarget64(rawVal.valInt);
}

/*!
 *	Convert a 32 bits float from host endianness to a frac16 in target endianness
 *	\param[in]	val						Value in host endianness in float.
 *	\return								Value in target endianness frac16.
 */
int16 sbgCanHostToTargetFrac16(float val)
{
	int16 newValue;

	//
	// We have to convert the float into frac16 format
	//
	if ( val > 1.0f)
	{
		newValue = 0x7FFF;
	}
	else if (val < -1.0f)
	{
		newValue = 0x8000;
	}
	else
	{
		newValue = (int16)(val * 32767.0f);
	}

	//
	// Handle endianess issues
	//
	return sbgCanHostToTarget16(newValue);
}

//----------------------------------------------------------------------//
//- Methods used to convert data from target to host format            -//
//----------------------------------------------------------------------//

/*!
 *	Convert a 16 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
int16 sbgCanTargetToHost16(int16 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN

		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap16(val);

	#else
		#error	sbgCanHostToTarget16: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a unsigned 16 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint16 sbgCanTargetToHostU16(uint16 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN

		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap16(val);

	#else
		#error	sbgCanHostToTarget16: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a 32 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
int32 sbgCanTargetToHost32(int32 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN

		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap32(val);

	#else
		#error	sbgCanHostToTarget32: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a unsigned 32 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint32 sbgCanTargetToHostU32(uint32 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN

		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap32(val);

	#else
		#error	sbgCanHostToTarget32: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a 64 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness (little endian).
 *	\return								Value in host endianness.
 */
int64 sbgCanTargetToHost64(int64 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN
		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap64(val);

	#else
		#error	sbgHostToTarget64: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a unsigned 64 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness (little endian).
 *	\return								Value in host endianness.
 */
uint64 sbgCanTargetToHostU64(uint64 val)
{
	#if defined SBG_PLATFORM_BIG_ENDIAN
		//
		// The target is in big endian so is the platform
		//
		return val;

	#elif defined SBG_PLATFORM_LITTLE_ENDIAN

		//
		// The target is in big endian and the platform is in little endian
		//
		return swap64(val);

	#else
		#error	sbgHostToTarget64: You have to define your platform endianness!
	#endif
}

/*!
 *	Convert a 32 bits number from target endianness to a float in host endianness.
 *	\param[in]	val						Value in target endianness in float.
 *	\return								Value in host endianness in float.
 */
float sbgCanTargetToHostFloat(uint32 val)
{
	SwapUnion32 rawVal;

	//
	// First handle endianess issues
	//
	rawVal.valInt = sbgCanTargetToHost32(val);

	return ((float)rawVal.valInt)/1048576.0f;
}

/*!
 *	Convert a 64 bit number in from target endianness into a double value in host endianness.
 *	\param[in]	val						The value in double/fixed in little/big endian to convert.
 *	\return								Value in host endianness in double.
 */
double sbgCanTargetToHostDouble(uint64 val)
{
	SwapUnion64 rawVal;

	//
	// First handle endianess issues
	//
	rawVal.valInt = sbgCanTargetToHost64(val);

	return ((double)rawVal.valInt)/4294967296.0;
}

/*!
 *	Convert a 16 bits frac16 number from target endianness to a float in host endianness.
 *	\param[in]	val						Value in target endianness in frac16.
 *	\return								Value in host endianness in float.
 */
float sbgCanTargetToHostFrac16(int16 val)
{
	int16 newValue;

	//
	// First handle endianess issues
	//
	newValue = sbgCanTargetToHost16(val);

	//
	// return the number as a float
	///
	return ((float)newValue)/32767.0f;
}
