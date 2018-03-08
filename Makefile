include CommonDefs.mk

.DEFAULT_GOAL := help

.PHONY: help apidoc

help:
	@echo "Usage:"
	@echo "  make help      show help message"
	@echo "  make apidoc    make api doc"
	@echo "  make opendoc   open api doc (html)"

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
