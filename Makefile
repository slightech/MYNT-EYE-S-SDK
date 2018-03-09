include CommonDefs.mk

.DEFAULT_GOAL := help

help:
	@echo "Usage:"
	@echo "  make help            show help message"
	@echo "  make apidoc          make api doc"
	@echo "  make opendoc         open api doc (html)"
	@echo "  make init            init project"
	@echo "  make test            build test and run"
	@echo "  make clean|cleanall  clean generated or useless things"

.PHONY: help

# doc

apidoc:
	@$(call echo,Make $@)
	@sh ./doc/build.sh

opendoc: apidoc
	@$(call echo,Make $@)
	@$(shell sh ./doc/langs.sh 1); \
	for lang in "$${LANGS[@]}"; do \
		html=./doc/output/$$lang/html/index.html; \
		[ -f "$$html" ] && sh ./scripts/open.sh $$html; \
	done

.PHONY: apidoc opendoc

# init

init:
	@$(call echo,Make $@)
	@sh ./scripts/init.sh

.PHONY: init

# deps

submodules:
	@git submodule update --init

third_party: submodules
	@$(call echo,Make $@)
	@$(call echo,Make glog,33)
	@$(call cmake_build,./third_party/glog/_build)

.PHONY: submodules third_party

# test

test: submodules
	@$(call echo,Make $@)
	@$(call echo,Make gtest,33)
	@$(call cmake_build,./tests/gtest/_build)

.PHONY: test

# clean

clean:
	@$(call echo,Make $@)
	@$(call rm,./tests/gtest/_build/)
	@$(call rm,./third_party/glog/_build/)

cleanall: clean
	@$(call rm,./doc/output/)
	@$(FIND) . -type f -name ".DS_Store" -print0 | xargs -0 rm -f

.PHONY: clean cleanall

# others

host:
	@$(call echo,Make $@)
	@echo HOST_OS: $(HOST_OS)
	@echo HOST_ARCH: $(HOST_ARCH)
	@echo ECHO: $(ECHO)
	@echo CC: $(CC)
	@echo CXX: $(CXX)
	@echo MAKE: $(MAKE)
	@echo BUILD: $(BUILD)
	@echo FIND: $(FIND)
	@echo LDD: $(LDD)
	@echo CMAKE: $(CMAKE)
	@#$(FIND) . -name READ*

.PHONY: host
