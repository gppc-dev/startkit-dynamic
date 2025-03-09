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

struct gppc_path
{
	const struct gppc_point *path; /// store array of size length. Allows NULL only if length=0.
	uint32_t length; /// length of the (x,y) pairs path.
	uint8_t incomplete; /// boolean for incomplete path, if 1 then contest will call gppc_get_path again.
};

/**
 * Array of a 2D table.  (0,0) is located at top-left corner.
 * Packed in a 1D bitarray, row-by-row.
 * Use `gppc_patch_get` or `gppc_patch_get_xy` on how to access grid cells.
 * 2D (x,y) point is converted to id in array: y * width + x
 * Data stored in `bitarray` as bits into 1-byte integers, then lsb ordering.
 * Example bit access for i: bitarray[i / 8] & (1 << i % 8), or see access functions.
 * gppc_patch_get(i) gives bit for index i, 1=traversable, 0=blocker.
 * 
 * pos is the location to apply the patch, more details in gppc_map_change.
 *
 * Any bitarray pointer lifetime guarantee until `gppc_free_data` is called and returns.
 * IMPORTANT: bitarray is constant and should not be modified, changing will affect client-side
 * validation but not server-side validation.
 */
struct gppc_patch
{
	const uint8_t *bitarray;
	uint16_t width;
	uint16_t height;
	struct gppc_point pos;
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
 * It will not be called in the same program execution as `gppc_search_init` is called,
 * all data must be shared in the preprocess_filename file.
 * 
 * Called with command below:
 * ./run -pre file.scen
 * 
 * @param[in] init_map The starting full map. width/height is the search grid's width/height, pos = (0,0).
 *                     Guarantee to be same to map given in `gppc_search_init` and map state valid in
 *                     first query from `gppc_get_path`.
 * @param[in] preprocess_filename The filename you write the preprocessing data to.  Open in write mode.
 *                                Guarantee that contents written to preprocess_filename is same content
 *                                as file given to `gppc_search_init` preprocess_filename.
 */
void gppc_preprocess_init_map(struct gppc_patch init_map, const char* preprocess_filename);


/**
 * User code used to setup search before queries.  Can also load pre-processing data from file preprocess_filename.
 * It will not be called in the same program execution as `gppc_preprocess_init_map` is called,
 * all data must be shared through file.
 * 
 * active_map holds the pointer to the entire initial map.
 * The pointer active_map.bitarray lifetime holds gppc_patch guarantee until after gppc_free_data returns.
 * active_map width/height is the maximum dimensions of any gppc_patch. pos = (0,0)
 * All dynamic changes to map will be applied to active_map.bitarray before call to gppc_map_change is made.
 * 
 * Return pointer is data the user can allocate on heap and share with other function calls.
 * This pointer should remain in scope until gppc_free_data, which should clean up.
 * 
 * Called with any commands below:
 * ./run -run file.map.scen
 * ./run -check file.map.scen
 * 
 * @param[in] active_map The starting full map.
 * @param[in] preprocess_filename The filename for the preprocessed data from gppc_preprocess_init_map, if using.  Open in read mode.
 * @return User data to be provided to other calls.
 */
void *gppc_search_init(struct gppc_patch active_map, const char* preprocess_filename);


/**
 * This will only be called when the map changes between queries.
 * It is guaranteed that active_map from gppc_search_init has already applied all patches
 * in changes.
 * 
 * changes provides a set of patch changes.  Each gppc_patch is applied by 
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
 * Returns user provided gppc_path containing path segment.
 * 
 * gppc_path.path must be allocated with enough size for gppc_path.length gpp_point's.
 * It is guaranteed that gppc_path.path pointer is only used outside this library until next
 * call to library header function.
 * User is required to manage this memory allocation, and is recommended to reuse for speed.
 * The User should free this allocation on call to gppc_free_search_data.
 * 
 * This function can be called multiple times for a query, based on user return values.
 * The gppc_path.incomplete=0 marks the final call for the query.
 * Every call to gppc_get_path will concatenate the path for a query, so the user
 * should only return additional path segments, not a prefix path to each call.
 * Users who return the whole path only just need to return the whole path.
 * The path must start with start on the first call and end with goal on the final call,
 * except in the case where no path exists.
 * The validator will be invoked after each gppc_get_path call.
 * 
 * When no path exists, you must return gppc_path with length=0 and incomplete=0.
 * Return any path where length=0 will be considered as a no path result, an incomplete=1
 * is invalid in this case and your program will error.
 * If previous parts of the path have already been returned before your algorithm
 * determined that no path is possible, those parts will be discarded and you will have
 * a correct no answer response.
 * 
 * Example for multi-called path:
 * Full path from start to goal: (2,4), (2,5), (3,5), (4,6), (5,6), (6,6)
 * First call: (2,4), (2,5)
 * Second call: (3,5), (4,6), (5,6)
 * Thrid call: (6,6)
 * The system will account for the jump between call's
 * E.g. second call will be treated as: (2,5), (3,5), (4,6), (5,6).
 * 
 * @param[in] data User data from gppc_search_init.
 * @param[in] start Query start location.
 * @param[in] goal Query goal location.
 * @return The next part (or whole) user-provided path. Set incomplete=0 once path is found.
 *         Set incomplete=0 and length=0 for no possible path.
 *         Can return gppc_path{} for a valid no possible path.
*/
struct gppc_path gppc_get_path(void *data, struct gppc_point start, struct gppc_point goal);


/**
 * Cleans up search data
*/
void gppc_free_data(void *data);


/**
 * @return Get entry name.
*/
const char* gppc_get_name();

#ifdef __cplusplus
}
#endif

#endif // GPPC_ENTRY_H
