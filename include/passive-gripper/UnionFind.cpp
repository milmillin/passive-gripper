#include "UnionFind.h"

#include <limits>

namespace psg {

UnionFind::UnionFind(size_t size)
    : parent(size, std::numeric_limits<size_t>::max()) {}

void UnionFind::merge(size_t a, size_t b) {
  size_t ancestor_a = find(a);
  size_t ancestor_b = find(b);
  if (ancestor_a != ancestor_b) {
    parent[ancestor_b] = ancestor_a;
  }
}

size_t UnionFind::find(size_t a) {
  if (parent[a] == std::numeric_limits<size_t>::max()) {
    return a;
  }
  return parent[a] = find(parent[a]);
}

}  // namespace psg
