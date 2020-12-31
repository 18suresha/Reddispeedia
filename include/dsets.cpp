#include "dsets.h"

void DisjointSets::addelements(int num) {
  for (int i = 0; i < num; ++i) {
    _elems.push_back(-1);
  }
}	

int DisjointSets::find(int elem) {
  if (_elems[elem] < 0) {
      return elem;
  }
  int i = find(_elems[elem]);
  _elems[elem] = i;
  return i;
}	


void DisjointSets::setunion(int a, int b) {
  int a_i = find(a);
  int b_i = find(b);
  if (a_i != b_i) {
    int tot_elem = _elems[a_i] + _elems[b_i];
    if (_elems[a_i] > _elems[b_i]) {
      _elems[a_i] = b_i;
      _elems[b_i] = tot_elem;
    } else {
      _elems[b_i] = a_i;
      _elems[a_i] = tot_elem;
    }
    _full = tot_elem * -1 == (int) _elems.size();
  }
}	


int DisjointSets::size(int elem) {
  return -1 * _elems[find(elem)];
}	

bool DisjointSets::isFull() {
  return _full || _elems.size() == 1;
}
