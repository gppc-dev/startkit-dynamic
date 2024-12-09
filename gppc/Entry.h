/*
Copyright (c) 2023 Grid-based Path Planning Competition and Contributors <https://gppc.search-conference.org/>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef GPPC_ENTRY_H
#define GPPC_ENTRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <assert.h>

struct gppc_point
{
	uint16_t x;
	uint16_t y;
};

struct gppc_patch
{
	const uint8_t *bitarray;
	uint16_t width;
	uint16_t height;
	struct gppc_point pos;
};

struct gppc_path
{
  const struct gppc_point *path; /// store array of size 2*len, path expected as pairs of x,y values. Allows NULL only if length=0.
  uint32_t length; /// length of the (x,y) pairs path
  uint8_t incomplete; /// boolean for incomplete path, if 1 then contest will call gppc_get_path again.
};

/**
 * @param map The patch to get data from
 * @param i The index where data resides
 * @return 0/1 -> 1=traversable, 0=blocker
 */
inline int gppc_patch_get(struct gppc_patch patch, int i)
{
	assert(0 <= i && i < (int)patch.width * (int)patch.height);
	return ( patch.bitarray[(i >> 3)] >> (i & 7) ) & 1;
}
inline int gppc_patch_get_xy(struct gppc_patch patch, uint16_t x, uint16_t y)
{
	assert(x < patch.width && y < patch.height);
	return gppc_patch_get(patch, (int)y * (int)patch.width + (int)x);
}

/**
 * User code used during preprocessing of a map.  Can be left blank if no pre-processing is required.
 * It will not be called in the same program execution as `PrepareForSearch` is called,
 * all data must be shared through file.
 * 
 * Called with command below:
 * ./run -pre file.scen
 * 
 * @param[in] init_map Array of 2D table.  (0,0) is located at top-left corner.
 *                     Packed as 1D array, row-by-ray, i.e. first width bool's give row y=0, next width y=1
 *                     init_map.bitarray[i] returns `true` if (x,y) is traversable, `false` otherwise.
 *                     init_map gives width/height of whole map.  init_map x/y are always 0 here.
 *                     See gppc_patch_get and gppc_patch_get_xy for more on how the data is arranged.
 * @param[in] preprocess_filename The filename you write the preprocessing data to.  Open in write mode.
 */
void gppc_preprocess_init_map(struct gppc_patch init_map, const char* preprocess_filename);


/**
 * User code used to setup search before queries.  Can also load pre-processing data from file to speed load.
 * It will not be called in the same program execution as `PreprocessMap` is called,
 * all data must be shared through file.
 * 
 * active_map holds the pointer to the current map state.  The pointer active_map.bitarray is guaranteed
 * to always be valid and up to date on every function call.
 * active_map is guaranteed to be the same as init_map called in gppc_preprocess_init_map.
 * 
 * Called with any commands below:
 * ./run -run file.map.scen
 * ./run -check file.map.scen
 * 
 * @param[in] active_map Array of 2D table.  (0,0) is located at top-left corner.
 *                       Packed as 1D array, row-by-ray, i.e. first width bool's give row y=0, next width y=1
 *                       init_map.bitarray[i] returns `true` if (x,y) is traversable, `false` otherwise.
 *                       init_map gives width/height of whole map.  init_map x/y are always 0 here.
 *                       See gppc_patch_get and gppc_patch_get_xy for more on how the data is arranged.
 * @param[in] preprocess_filename The filename for the preprocessed data on init_map, if using.  Open in read mode.
 * @return User data to be provided to other calls.
 */
void *gppc_search_init(struct gppc_patch active_map, const char* preprocess_filename);


/**
 * This will only be called when the map changes between queries.
 * It is guaranteed that active_map from gppc_search_init has already applied all patches
 * in changes.
 * 
 * Called with any commands below:
 * ./run -run file.map file.map.scen
 * ./run -check file.map file.map.scen
 * 
 * @param[in] data User data from gppc_search_init.
 * @param[in] changes Array pointer of patch of all changes.
 * @param[in] changes_length The filename for the preprocessed data on init_map, if using.  Open in read mode.
 * @return User data to be provided to other calls.
 */
void gppc_map_change(void *data, const struct gppc_patch* changes, uint32_t changes_length);


/**
 * Find the path from (sx,sy) to (gx,gy).
 * Returns user provided gppc_path.
 * 
 * gppc_path.path must be allocated with enough size for 2*gppc_path.length.
 * It is guaranteed that the only gppc_path used outside this function will be the last
 * call of this function.
 * User is required to manage this memory allocation, and is recommended to reuse for speed.
 * The User should free this allocation on call to gppc_free_search_data.
 * 
 * @param[in] data User data from gppc_search_init.
 * @param[in] sx,sy Starting x/y point.
 * @param[in] gx,gy Goal x/y point.
 * @return The user-provided path. Set incomplete=0 once path is found. Set length=0 for no possible path.
 *         Can return gppc_path{} for valid no possible path.
*/
struct gppc_path gppc_get_path(void *data, struct gppc_point start, struct gppc_point goal);


/**
 * Cleans up search data.
*/
void gppc_free_data(void *data);


/**
 * @return Get entry name.  Must be valid until gppc_free_data is called.  Should be a string-literal.
*/
const char* gppc_get_name();

#ifdef __cplusplus
}
#endif

#endif // GPPC_ENTRY_H
