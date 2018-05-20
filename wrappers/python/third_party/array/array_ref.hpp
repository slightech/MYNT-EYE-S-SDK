// From Praetorian's answer here: http://stackoverflow.com/a/27560620
#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <type_traits>

template <typename T, bool NullTerminated = false>
class array_ref;

namespace {

template <typename T>
struct base_type {
  using type = T;
};

template <typename T, bool b>
struct base_type<array_ref<T, b>> : base_type<T> {};

template <typename T>
using BaseType = typename base_type<T>::type;

template <typename T>
struct pointer_type {
  using type = T *;
};

template <typename T, bool b>
struct pointer_type<array_ref<T, b>> {
  using type = typename pointer_type<T>::type *;
};

template <typename T>
using PointerType = typename pointer_type<T>::type;
}

/**
 * array_ref offers a non-const view into an array. The storage for the array is
 * not owned by the
 * array_ref object, and it is the client's responsibility to ensure the backing
 * store reamins
 * alive while the array_ref object is in use.
 *
 * @tparam T
 *      Type of elements in the array
 */
template <typename T>
struct array_ref_base {
 public:
  /** Alias for the type of elements in the array */
  using value_type = T;
  /** Alias for a pointer to value_type */
  using pointer = T *;
  /** Alias for a constant pointer to value_type */
  using const_pointer = T const *;
  /** Alias for a reference to value_type */
  using reference = T &;
  /** Alias for a constant reference to value_type */
  using const_reference = T const &;
  /** Alias for an unsigned integral type used to represent size related values
   */
  using size_type = std::size_t;
  /** Alias for a signed integral type used to represent result of difference
   * computations */
  using difference_type = std::ptrdiff_t;
  /** Default constructor */
};

template <typename T>
bool operator==(const array_ref_base<T> &ref1, const array_ref_base<T> &ref2) {
  return &ref1 == &ref2;
}

template <typename T, bool NullTerminated>
class array_ref : public array_ref_base<T> {
 public:
  using typename array_ref_base<T>::value_type;
  using typename array_ref_base<T>::pointer;
  using typename array_ref_base<T>::const_pointer;
  using typename array_ref_base<T>::reference;
  using typename array_ref_base<T>::const_reference;
  using typename array_ref_base<T>::size_type;
  using typename array_ref_base<T>::difference_type;
  /** Alias for an iterator pointing at value_type objects */
  using iterator = T *;
  /** Alias for a constant iterator pointing at value_type objects */
  using const_iterator = T const *;
  /** Alias for a reverse iterator pointing at value_type objects */
  using reverse_iterator = std::reverse_iterator<iterator>;
  /** Alias for a constant reverse iterator pointing at value_type objects */
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  constexpr array_ref() noexcept = default;

  template <bool b = NullTerminated,
            typename = typename std::enable_if<b, void>::type>
  array_ref(pointer first) noexcept : begin_{first}, length_{} {
    while (begin_ && begin_[length_++])
      ;
  }

  /**
   * Constructor that accepts a pointer to an array and the number of elements
   * pointed at
   *
   * @param arr
   *    Pointer to array
   * @param length
   *    Number of elements pointed at
   */
  constexpr array_ref(pointer arr, size_type length) noexcept
      : begin_{arr},
        length_{length} {}

  array_ref(
      pointer arr, const std::vector<size_type> &lengths,
      std::size_t size_index = 0)
      : begin_{arr}, length_{lengths[size_index]} {}

  /**
   * Constructor that accepts a reference to an array
   *
   * @tparam N
   *    Number of elements in the array
   */
  template <size_type N>
  constexpr array_ref(T (&arr)[N]) noexcept : begin_{&arr[0]}, length_{N} {}

  /**
   * Constructor taking a pair of pointers pointing to the first element and one
   * past the last
   * element of the array, respectively.
   *
   * @param first
   *    Pointer to the first element of the array
   * @param last
   *    Pointer to one past the last element of the array
   */
  array_ref(pointer first, pointer last) noexcept
      : begin_{first},
        length_{static_cast<size_type>(std::distance(first, last))} {}

  /** Copy constructor */
  constexpr array_ref(array_ref const &) noexcept = default;

  /** Copy assignment operator */
  array_ref &operator=(array_ref const &) noexcept = default;

  /** Move constructor */
  constexpr array_ref(array_ref &&) noexcept = default;

  /** Move assignment operator */
  array_ref &operator=(array_ref &&) noexcept = default;

  /**
   * Returns an iterator to the first element of the array. If the array is
   * empty, the
   * returned iterator will be equal to end().
   *
   * @return An iterator to the first element of the array
   */
  /*constexpr*/ iterator begin() noexcept {
    return begin_;
  }

  /**
   * Returns a constant iterator to the first element of the array. If the array
   * is empty, the
   * returned iterator will be equal to end().
   *
   * @return A constant iterator to the first element of the array
   */
  constexpr const_iterator begin() const noexcept {
    return begin_;
  }

  /**
   * Returns a constant iterator to the first element of the array. If the array
   * is empty, the
   * returned iterator will be equal to end().
   *
   * @return A constant iterator to the first element of the array
   */
  constexpr const_iterator cbegin() const noexcept {
    return begin_;
  }

  /**
   * Returns an iterator to the element following the last element of the array.
   *
   * @return An iterator to the element following the last element of the array
   */
  /*constexpr*/ iterator end() noexcept {
    return begin() + size();
  }

  /**
   * Returns a constant iterator to the element following the last element of
   * the array.
   *
   * @return A constant iterator to the element following the last element of
   * the array
   */
  constexpr const_iterator end() const noexcept {
    return begin() + size();
  }

  /**
   * Returns a constant iterator to the element following the last element of
   * the array.
   *
   * @return A constant iterator to the element following the last element of
   * the array
   */
  constexpr const_iterator cend() const noexcept {
    return cbegin() + size();
  }

  /**
   * Returns a reverse iterator to the first element of the reversed array. It
   * corresponds to the
   * last element of the non-reversed array.
   *
   * @return A reverse iterator to the first element of the reversed array
   */
  reverse_iterator rbegin() noexcept {
    return reverse_iterator(end());
  }

  /**
   * Returns a constant reverse iterator to the first element of the reversed
   * array. It corresponds
   * to the last element of the non-reversed array.
   *
   * @return A constant reverse iterator to the first element of the reversed
   * array
   */
  const_reverse_iterator rbegin() const noexcept {
    return const_reverse_iterator(cend());
  }

  /**
   * Returns a constant reverse iterator to the first element of the reversed
   * array. It corresponds
   * to the last element of the non-reversed array.
   *
   * @return A constant reverse iterator to the first element of the reversed
   * array
   */
  const_reverse_iterator crbegin() const noexcept {
    return const_reverse_iterator(cend());
  }

  /**
   * Returns a reverse iterator to the element following the last element of the
   * reversed array. It
   * corresponds to the element preceding the first element of the non-reversed
   * array.
   *
   * @return A reverse iterator to the element following the last element of the
   * reversed array
   */
  reverse_iterator rend() noexcept {
    return reverse_iterator(begin());
  }

  /**
   * Returns a constant reverse iterator to the element following the last
   * element of the reversed
   * array. It corresponds to the element preceding the first element of the
   * non-reversed array.
   *
   * @return A constant reverse iterator to the element following the last
   * element of the reversed
   *         array
   */
  const_reverse_iterator rend() const noexcept {
    return const_reverse_iterator(cbegin());
  }

  /**
   * Returns a constant reverse iterator to the element following the last
   * element of the reversed
   * array. It corresponds to the element preceding the first element of the
   * non-reversed array.
   *
   * @return A constant reverse iterator to the element following the last
   * element of the reversed
   *         array
   */
  const_reverse_iterator crend() const noexcept {
    return const_reverse_iterator(cbegin());
  }

  /**
   * Returns the number of elements in the array.
   *
   * @return The number of elements in the array
   */
  constexpr size_type size() const noexcept {
    return length_;
  }

  /**
   * Indicates whether the array has no elements
   *
   * @return true if the array has no elements, false otherwise
   */
  constexpr bool empty() const noexcept {
    return size() == 0;
  }

  /**
   * Returns a reference to the element at the specified location.
   *
   * @return A reference to the element at the specified location
   * @pre i < size()
   */
  /*constexpr*/ reference operator[](size_type i) {
#ifndef NDEBUG
    return at(i);
#else
    return *(begin() + i);
#endif
  }

  /**
   * Returns a constant reference to the element at the specified location.
   *
   * @return A constant reference to the element at the specified location
   * @pre i < size()
   */
  constexpr const_reference operator[](size_type i) const {
#ifndef NDEBUG
    return at(i);
#else
    return *(begin() + i);
#endif
  }

  /**
   * Returns a reference to the element at the specified location, with bounds
   * checking.
   *
   * @return A reference to the element at the specified location
   * @throw std::out_of_range if the specified index is not within the range of
   * the array
   */
  /*constexpr*/ reference at(size_type i) {
    if (i >= size()) {
      throw std::out_of_range("index out of range");
    }
    return *(begin() + i);
  }

  /**
   * Returns a constant reference to the element at the specified location, with
   * bounds checking.
   *
   * @return A constant reference to the element at the specified location
   * @throw std::out_of_range if the specified index is not within the range of
   * the array
   */
  /*constexpr*/ const_reference at(size_type i) const {
    if (i >= size()) {
      throw std::out_of_range("index out of range");
    }
    return *(begin() + i);
  }

  /**
   * Returns a reference to the first element of the array
   *
   * @return A reference to the first element of the array
   * @pre empty() == false
   */
  /*constexpr*/ reference front() noexcept {
    return *begin();
  }

  /**
   * Returns a reference to the first element of the array
   *
   * @return A reference to the first element of the array
   * @pre empty() == false
   */
  constexpr const_reference front() const noexcept {
    return *begin();
  }

  /**
   * Returns a reference to the last element of the array
   *
   * @return A reference to the last element of the array
   * @pre empty() == false
   */
  /*constexpr*/ reference back() noexcept {
    return *(end() - 1);
  }

  /**
   * Returns a constant reference to the last element of the array
   *
   * @return A constant reference to the last element of the array
   * @pre empty() == false
   */
  constexpr const_reference back() const noexcept {
    return *(end() - 1);
  }

  /**
   * Returns a pointer to the address of the first element of the array
   *
   * @return A pointer to the address of the first element of the array
   */
  /*constexpr*/ pointer data() noexcept {
    return begin();
  }

  /**
   * Returns a constant pointer to the address of the first element of the array
   *
   * @return A constant pointer to the address of the first element of the array
   */
  constexpr const_pointer data() const noexcept {
    return begin();
  }

  /**
   * Resets the operand back to its default constructed state
   *
   * @post empty() == true
   */
  void clear() noexcept {
    begin_ = nullptr;
    length_ = 0;
  }

 protected:
  /** Pointer to the first element of the referenced array */
  pointer begin_ = nullptr;
  /** Number of elements in the referenced array */
  size_type length_ = size_type();
};

// template<typename T>
// class null_terminated_array_ref : public array_ref<T> {
//     public:
//         using pointer = typename array_ref<T>::pointer;
//         using size_type = typename array_ref<T>::size_type;
//
//
//         /** Default constructor */
//         constexpr null_terminated_array_ref() noexcept : array_ref<T>{} {}
//
//         /**
//          * Constructor that accepts a pointer to an array and the number of
//          elements pointed at
//          *
//          * @param arr
//          *    Pointer to array
//          * @param length
//          *    Number of elements pointed at
//          */
//         constexpr null_terminated_array_ref( pointer arr, size_type length )
//         : array_ref<T>{arr, length} {}
//
//         /**
//          * Constructor that accepts a reference to an array
//          *
//          * @tparam N
//          *    Number of elements in the array
//          */
//         template<size_type N>
//             constexpr null_terminated_array_ref( T (&arr)[N] ) noexcept :
//             array_ref<T>{arr} {}
//
//         /**
//          * Constructor taking a pair of pointers pointing to the first
//          element and one past the last
//          * element of the array, respectively.
//          *
//          * @param first
//          *    Pointer to the first element of the array
//          * @param last
//          *    Pointer to one past the last element of the array
//          */
//         null_terminated_array_ref( pointer first, pointer last ) noexcept :
//         array_ref<T>{first, last} {}
//
//         /** Copy constructor */
//         constexpr null_terminated_array_ref( null_terminated_array_ref const&
//         ) noexcept = default;
//
//         /** Copy assignment operator */
//         null_terminated_array_ref& operator=( null_terminated_array_ref
//         const& ) noexcept = default;
//
//         /** Move constructor */
//         constexpr null_terminated_array_ref( null_terminated_array_ref&& )
//         noexcept = default;
//
//         /** Move assignment operator */
//         null_terminated_array_ref& operator=( null_terminated_array_ref&& )
//         noexcept = default;
//
// };

template <typename T, bool b>
class array_ref<array_ref<T, b>> : public array_ref_base<array_ref<T, b>> {
  using internal_type = BaseType<array_ref<T, b>>;
  using internal_pointer = PointerType<array_ref<T, b>>;

 protected:
  std::vector<array_ref<T, b>> vec_;

 public:
  using typename array_ref_base<array_ref<T, b>>::value_type;
  using typename array_ref_base<array_ref<T, b>>::pointer;
  using typename array_ref_base<array_ref<T, b>>::const_pointer;
  using typename array_ref_base<array_ref<T, b>>::reference;
  using typename array_ref_base<array_ref<T, b>>::const_reference;
  using typename array_ref_base<array_ref<T, b>>::size_type;
  using typename array_ref_base<array_ref<T, b>>::difference_type;

  /** Alias for an iterator pointing at value_type objects */
  using iterator = typename std::vector<array_ref<T, b>>::iterator;
  /** Alias for a constant iterator pointing at value_type objects */
  using const_iterator = typename std::vector<array_ref<T, b>>::const_iterator;
  /** Alias for a reverse iterator pointing at value_type objects */
  using reverse_iterator =
      typename std::vector<array_ref<T, b>>::reverse_iterator;
  /** Alias for a constant reverse iterator pointing at value_type objects */
  using const_reverse_iterator =
      typename std::vector<array_ref<T, b>>::const_reverse_iterator;

  /** Default constructor */
  constexpr array_ref() noexcept = default;

  array_ref(internal_pointer arr, const size_type length) {
    for (size_type i = 0; i < length; ++i) {
      vec_.emplace_back(arr[i]);
    }
  }

  array_ref(
      internal_pointer arr, const std::vector<size_type> &lengths,
      std::size_t size_index = 0) {
    for (size_type i = 0; i < lengths[size_index]; ++i) {
      if (size_index < lengths.size()) {
        vec_.emplace_back(arr[i], lengths, size_index + 1);
      } else {
        vec_.emplace_back(arr[i]);
      }
    }
  }

  template <typename InputIt>
  array_ref(InputIt first, InputIt last) : vec_{first, last} {}

  /**
   * Constructor that accepts a reference to an array
   *
   * @tparam N
   *    Number of elements in the array
   */
  template <typename A, std::size_t N>
  array_ref(A (&arr)[N]) {
    for (size_type i = 0; i < N; ++i) {
      vec_.emplace_back(arr[i]);
    }
  }

  /** Copy constructor */
  array_ref(array_ref const &) = default;

  /** Copy assignment operator */
  array_ref &operator=(array_ref const &) = default;

  /** Move constructor */
  constexpr array_ref(array_ref &&) noexcept = default;

  /** Move assignment operator */
  array_ref &operator=(array_ref &&) noexcept = default;

  /**
   * Returns an iterator to the first element of the array. If the array is
   * empty, the
   * returned iterator will be equal to end().
   *
   * @return An iterator to the first element of the array
   */
  /*constexpr*/ iterator begin() noexcept {
    return vec_.begin();
  }

  /**
   * Returns a constant iterator to the first element of the array. If the array
   * is empty, the
   * returned iterator will be equal to end().
   *
   * @return A constant iterator to the first element of the array
   */
  constexpr const_iterator begin() const noexcept {
    return vec_.cbegin();
  }

  /**
   * Returns a constant iterator to the first element of the array. If the array
   * is empty, the
   * returned iterator will be equal to end().
   *
   * @return A constant iterator to the first element of the array
   */
  constexpr const_iterator cbegin() const noexcept {
    return vec_.cbegin();
  }

  /**
   * Returns an iterator to the element following the last element of the array.
   *
   * @return An iterator to the element following the last element of the array
   */
  /*constexpr*/ iterator end() noexcept {
    return vec_.end();
  }

  /**
   * Returns a constant iterator to the element following the last element of
   * the array.
   *
   * @return A constant iterator to the element following the last element of
   * the array
   */
  constexpr const_iterator end() const noexcept {
    return vec_.cend();
  }

  /**
   * Returns a constant iterator to the element following the last element of
   * the array.
   *
   * @return A constant iterator to the element following the last element of
   * the array
   */
  constexpr const_iterator cend() const noexcept {
    return vec_.cend();
  }

  /**
   * Returns a reverse iterator to the first element of the reversed array. It
   * corresponds to the
   * last element of the non-reversed array.
   *
   * @return A reverse iterator to the first element of the reversed array
   */
  reverse_iterator rbegin() noexcept {
    return vec_.rbegin();
  }

  /**
   * Returns a constant reverse iterator to the first element of the reversed
   * array. It corresponds
   * to the last element of the non-reversed array.
   *
   * @return A constant reverse iterator to the first element of the reversed
   * array
   */
  const_reverse_iterator rbegin() const noexcept {
    return vec_.crbegin();
  }

  /**
   * Returns a constant reverse iterator to the first element of the reversed
   * array. It corresponds
   * to the last element of the non-reversed array.
   *
   * @return A constant reverse iterator to the first element of the reversed
   * array
   */
  const_reverse_iterator crbegin() const noexcept {
    return vec_.crbegin();
  }

  /**
   * Returns a reverse iterator to the element following the last element of the
   * reversed array. It
   * corresponds to the element preceding the first element of the non-reversed
   * array.
   *
   * @return A reverse iterator to the element following the last element of the
   * reversed array
   */
  reverse_iterator rend() noexcept {
    return vec_.rend();
  }

  /**
   * Returns a constant reverse iterator to the element following the last
   * element of the reversed
   * array. It corresponds to the element preceding the first element of the
   * non-reversed array.
   *
   * @return A constant reverse iterator to the element following the last
   * element of the reversed
   *         array
   */
  const_reverse_iterator rend() const noexcept {
    return vec_.crend();
  }

  /**
   * Returns a constant reverse iterator to the element following the last
   * element of the reversed
   * array. It corresponds to the element preceding the first element of the
   * non-reversed array.
   *
   * @return A constant reverse iterator to the element following the last
   * element of the reversed
   *         array
   */
  const_reverse_iterator crend() const noexcept {
    return vec_.crend();
  }

  /**
   * Returns the number of elements in the array.
   *
   * @return The number of elements in the array
   */
  constexpr size_type size() const noexcept {
    return vec_.size();
  }

  /**
   * Indicates whether the array has no elements
   *
   * @return true if the array has no elements, false otherwise
   */
  constexpr bool empty() const noexcept {
    return vec_.empty();
  }

  /**
   * Returns a reference to the element at the specified location.
   *
   * @return A reference to the element at the specified location
   * @pre i < size()
   */
  /*constexpr*/ reference operator[](size_type i) {
    return vec_[i];
  }

  /**
   * Returns a constant reference to the element at the specified location.
   *
   * @return A constant reference to the element at the specified location
   * @pre i < size()
   */
  constexpr const_reference operator[](size_type i) const {
    return vec_[i];
  }

  /**
   * Returns a reference to the element at the specified location, with bounds
   * checking.
   *
   * @return A reference to the element at the specified location
   * @throw std::out_of_range if the specified index is not within the range of
   * the array
   */
  /*constexpr*/ reference at(size_type i) {
    return vec_.at(i);
  }

  /**
   * Returns a constant reference to the element at the specified location, with
   * bounds checking.
   *
   * @return A constant reference to the element at the specified location
   * @throw std::out_of_range if the specified index is not within the range of
   * the array
   */
  /*constexpr*/ const_reference at(size_type i) const {
    return vec_.at(i);
  }

  /**
   * Returns a reference to the first element of the array
   *
   * @return A reference to the first element of the array
   * @pre empty() == false
   */
  /*constexpr*/ reference front() noexcept {
    return vec_.front();
  }

  /**
   * Returns a reference to the first element of the array
   *
   * @return A reference to the first element of the array
   * @pre empty() == false
   */
  constexpr const_reference front() const noexcept {
    return vec_.front();
  }

  /**
   * Returns a reference to the last element of the array
   *
   * @return A reference to the last element of the array
   * @pre empty() == false
   */
  /*constexpr*/ reference back() noexcept {
    return vec_.back();
  }

  /**
   * Returns a constant reference to the last element of the array
   *
   * @return A constant reference to the last element of the array
   * @pre empty() == false
   */
  constexpr const_reference back() const noexcept {
    return vec_.back();
  }

  /**
   * Returns a pointer to the address of the first element of the array
   *
   * @return A pointer to the address of the first element of the array
   */
  /*constexpr*/ pointer data() noexcept {
    return vec_.data();
  }

  /**
   * Returns a constant pointer to the address of the first element of the array
   *
   * @return A constant pointer to the address of the first element of the array
   */
  constexpr const_pointer data() const noexcept {
    return vec_.data();
  }

  /**
   * Resets the operand back to its default constructed state
   *
   * @post empty() == true
   */
  void clear() noexcept {
    vec_.clear();
  }
};

// template<typename T, bool b>
// bool operator==(const array_ref<T, b>& ref1, const array_ref<T, b>& ref2) {
//     return &ref1 == &ref2;
// }
