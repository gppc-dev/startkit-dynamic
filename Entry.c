#include "Entry.h"
#include <stdlib.h>

void gppc_preprocess_init_map(struct gppc_patch init_map, const char* preprocess_filename)
{
	// PREPROCESSING IMPLEMENT
}


void *gppc_search_init(struct gppc_patch active_map, const char* preprocess_filename)
{
	// SEARCH SETUP IMPLEMENT
	abort();
}


void gppc_map_change(void *data, const struct gppc_patch* changes, uint32_t changes_length)
{
	// ON MAP CHANGE IMPLEMENT
}


struct gppc_path gppc_get_path(void *data, struct gppc_point start, struct gppc_point goal)
{
	// QUERY IMPLEMENT
	abort();
}


void gppc_free_data(void *data)
{
	// FREE IMPLEMENT
}


const char* gppc_get_name()
{
	abort();
	return "NAME_IMPLEMENT";
}
