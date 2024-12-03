#include "Entry.h"
#include "BaselineSearch.hxx"


void gppc_preprocess_init_map(gppc_patch init_map, const char* preprocess_filename)
{}


void *gppc_search_init(gppc_patch active_map, const char* preprocess_filename)
{
  auto* STS = new baseline::SpanningTreeSearch(active_map);
  return STS;
}


void gppc_map_change(void *data, gppc_patch* changes, uint32_t changes_length)
{
  auto* STS = static_cast<baseline::SpanningTreeSearch*>(data);
  // SpanningTreeSearch must update its internal structure.
  // It is not smart, so it will update the whole map, thus ignores changes.
  STS->update_grid();
}


gppc_path gppc_get_path(void *data, uint16_t sx, uint16_t sy, uint16_t gx, uint16_t gy)
{
  auto* STS = static_cast<baseline::SpanningTreeSearch*>(data);
  bool exists = STS->search(baseline::Point(sx, sy), baseline::Point(gx, gy));
  if (!exists)
    return gppc_path{};
  
  auto& path = STS->get_path();
  gppc_path res_path{};
  res_path.path = path.data();
  res_path.length = path.size();
  // res_path.incomplete = 0; // not required as value-init defaults it to 0
  return res_path;
}


void gppc_free_data(void *data)
{
  auto* STS = static_cast<baseline::SpanningTreeSearch*>(data);
  delete STS;
}


const char* gppc_get_name()
{
  return "example-DynamicSpanningTreeSearch-8N";
}
