#include "Entry.h"
#include <assert.h>

void gppc_preprocess_init_map(struct gppc_patch init_map, const char* preprocess_filename)
{
	// PREPROCESSING IMPLEMENT
}


void *gppc_search_init(struct gppc_patch active_map, const char* preprocess_filename)
{
	assert(0);
	// SEARCH SETUP IMPLEMENT
}


void gppc_map_change(void *data, const struct gppc_patch* changes, uint32_t changes_length)
{
	// ON MAP CHANGE IMPLEMENT
}


struct gppc_path gppc_get_path(void *data, uint16_t sx, uint16_t sy, uint16_t gx, uint16_t gy)
{
	assert(0);
	// QUERY IMPLEMENT
}


void gppc_free_search_data(void *data)
{
	// FREE IMPLEMENT
}


const char* gppc_get_name()
{
	assert(0);
	return "NAME_IMPLEMENT";
}
