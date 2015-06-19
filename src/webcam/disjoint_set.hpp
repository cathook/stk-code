#ifndef WEBCAM_DISJOINT_SET_HPP
#define WEBCAM_DISJOINT_SET_HPP

#include <vector>


namespace webcam {

class DisjointSet {
 public:
  DisjointSet(size_t size) : parent_(size) {
    for (size_t i = 0; i < size; ++i) {
      parent_[i] = i;
    }
  }

  void Union(size_t a, size_t b) {
    a = GetRoot(a);
    b = GetRoot(b);
    parent_[a] = b;
  }

  size_t GetRoot(size_t a) const {
    if (parent_[a] != a) {
      parent_[a] = GetRoot(parent_[a]);
    }
    return parent_[a];
  }

 private:
  mutable std::vector<size_t> parent_;
};

}  // namespace webcam

#endif  // WEBCAM_DISJOINT_SET_HPP
