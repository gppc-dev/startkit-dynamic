#include "Entry.h"
#include <cstdlib>


void gppc_preprocess_init_map(gppc_patch init_map, const char* preprocess_filename)
{
	// PREPROCESSING IMPLEMENT
}


void *gppc_search_init(gppc_patch active_map, const char* preprocess_filename)
{
	// SEARCH SETUP IMPLEMENT
	std::abort();
}


void gppc_map_change(void *data, const gppc_patch* changes, uint32_t changes_length)
{
	// ON MAP CHANGE IMPLEMENT
}


gppc_path gppc_get_path(void *data, gppc_point start, gppc_point goal)
{
	// QUERY IMPLEMENT
	std::abort();
}


void gppc_free_data(void *data)
{
	// FREE IMPLEMENT
}


const char* gppc_get_name()
{
	std::abort();
	return "NAME_IMPLEMENT";
}
