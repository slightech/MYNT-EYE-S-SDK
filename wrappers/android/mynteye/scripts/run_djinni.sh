#! /usr/bin/env bash
set -e
shopt -s nullglob

base_dir=$(cd "$(dirname "$0")" && pwd)

# options

while getopts "d:" opt; do
  case "$opt" in
    d) djinni_dir="$OPTARG" ;;
    ?) echo "Usage: $0 <-d DJINNI_DIR>"
       exit 2 ;;
  esac
done

if [ -z "$djinni_dir" ]; then
  echo "<-d DJINNI_DIR> option is required" 1>&2
  exit 2
fi

# generate

djinni_run="$djinni_dir/src/run-assume-built"
if [ ! -x "$djinni_run" ]; then
  echo "djinni run file not found: $djinni_run" 1>&2
  exit 2
fi

temp_out="$base_dir/djinni-output-temp"

java_package="com.slightech.mynteye"
cpp_namespace="mynteye_jni"

[ ! -e "$temp_out" ] || rm -r "$temp_out"

djinni_build() {
  local in_idl="$1"; shift
  "$djinni_run" \
  --java-out "$temp_out/java" \
  --java-package "$java_package" \
  --java-class-access-modifier "public" \
  --java-generate-interfaces true \
  --java-nullable-annotation "androidx.annotation.Nullable" \
  --java-nonnull-annotation "androidx.annotation.NonNull" \
  --ident-java-field mFooBar \
  \
  --cpp-out "$temp_out/cpp" \
  --cpp-namespace "$cpp_namespace" \
  --ident-cpp-enum-type FooBar \
  --ident-cpp-method FooBar \
  \
  --jni-out "$temp_out/jni" \
  --ident-jni-class NativeFooBar \
  --ident-jni-file NativeFooBar \
  \
  --yaml-out $(dirname "$in_idl") \
  --yaml-out-file "$(basename "$in_idl" .djinni).yaml" \
  \
  --idl "$in_idl"
}

djinni_build "$base_dir/mynteye_types.djinni"
djinni_build "$base_dir/mynteye.djinni"

# copy

mirror() {
  local prefix="$1" ; shift
  local src="$1" ; shift
  local dest="$1" ; shift
  mkdir -p "$dest"
  rsync -r --delete --checksum --itemize-changes "$src"/ "$dest" | sed "s/^/[$prefix]/"
}

dst_dir="$base_dir/../libmynteye/src/main"
cpp_out="$dst_dir/cpp/mynteye/cpp"
jni_out="$dst_dir/cpp/mynteye/jni"
java_out="$dst_dir/java/com/slightech/mynteye"

gen_stamp="$temp_out/gen.stamp"

echo "Copying generated code to final directories..."
mirror "cpp" "$temp_out/cpp" "$cpp_out"
mirror "jni" "$temp_out/jni" "$jni_out"
mirror "java" "$temp_out/java" "$java_out"

date > "$gen_stamp"

echo "djinni completed."
