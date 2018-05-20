// From Praetorian's answer here: http://stackoverflow.com/a/27560620
#include <boost/python.hpp>
#include <boost/python/suite/indexing/indexing_suite.hpp>

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <type_traits>

// Forward declaration
template <typename Array, bool NoProxy, typename DerivedPolicies>
class array_indexing_suite;

namespace detail {

template <typename Array, bool NoProxy>
struct final_array_derived_policies
    : array_indexing_suite<Array, NoProxy,
                           final_array_derived_policies<Array, NoProxy>> {};

} /* namespace detail */

template <typename Array,
          bool NoProxy = std::is_arithmetic<typename Array::value_type>::value,
          typename DerivedPolicies =
              detail::final_array_derived_policies<Array, NoProxy>>
class array_indexing_suite
    : public boost::python::indexing_suite<Array, DerivedPolicies, NoProxy> {
 public:
  typedef typename Array::value_type data_type;
  typedef typename Array::value_type key_type;
  typedef typename Array::size_type index_type;
  typedef typename Array::size_type size_type;
  typedef typename Array::difference_type difference_type;

  static data_type &get_item(Array &arr, index_type i) {
    return arr[i];
  }

  static void set_item(Array &arr, index_type i, data_type const &v) {
    arr[i] = v;
  }

  static void delete_item(Array & /*arr*/, index_type /*i*/) {
    ::PyErr_SetString(::PyExc_TypeError, "Cannot delete array item");
    boost::python::throw_error_already_set();
  }

  static size_type size(Array &arr) {
    return arr.size();
  }

  static bool contains(Array &arr, key_type const &key) {
    return std::find(arr.cbegin(), arr.cend(), key) != arr.cend();
  }

  static index_type get_min_index(Array &) {
    return 0;
  }

  static index_type get_max_index(Array &arr) {
    return arr.size();
  }

  static bool compare_index(Array &, index_type a, index_type b) {
    return a < b;
  }

  static index_type convert_index(Array &arr, PyObject *i_) {
    boost::python::extract<long> i(i_);
    if (i.check()) {
      long index = i();

      if (index < 0) {
        index += static_cast<decltype(index)>(DerivedPolicies::size(arr));
      }

      if ((index >= long(arr.size())) || (index < 0)) {
        ::PyErr_SetString(::PyExc_IndexError, "Index out of range");
        boost::python::throw_error_already_set();
      }
      return index;
    }

    ::PyErr_SetString(::PyExc_TypeError, "Invalid index type");
    boost::python::throw_error_already_set();
    return index_type();
  }

  static boost::python::object get_slice(
      Array &arr, index_type from, index_type to) {
    if (from > to) {
      return boost::python::object(Array());
    }
    return boost::python::object(Array(arr.begin() + from, arr.begin() + to));
  }

  static void set_slice(
      Array &arr, index_type from, index_type to, data_type const &v) {
    if (from > to) {
      return;

    } else if (to > arr.size()) {
      ::PyErr_SetString(::PyExc_IndexError, "Index out of range");
      boost::python::throw_error_already_set();

    } else {
      std::fill(arr.begin() + from, arr.begin() + to, v);
    }
  }

  template <typename Iter>
  static void set_slice(
      Array &arr, index_type from, index_type to, Iter first, Iter last) {
    auto num_items = std::distance(first, last);

    if ((from + num_items) > arr.size()) {
      ::PyErr_SetString(::PyExc_IndexError, "Index out of range");
      boost::python::throw_error_already_set();
      return;
    }

    if (from > to) {
      std::copy(first, last, arr.begin() + from);

    } else {
      if (static_cast<decltype(num_items)>(to - from) != num_items) {
        ::PyErr_SetString(::PyExc_TypeError, "Array length is immutable");
        boost::python::throw_error_already_set();
        return;
      }

      std::copy(first, last, arr.begin() + from);
    }
  }

  static void delete_slice(
      Array & /*arr*/, index_type /*from*/, index_type /*to*/) {
    ::PyErr_SetString(::PyExc_TypeError, "Cannot delete array item(s)");
    boost::python::throw_error_already_set();
  }
};
