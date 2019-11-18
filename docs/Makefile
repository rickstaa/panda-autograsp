## Make file for the panda_autograsp sphinx documentation

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SOURCEDIR     = source
BUILDDIR      = build
GH_PAGES_SOURCES = docs panda_autograsp

# Put it first so that "make" without argument is like "make help".
.PHONY: help Makefile
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: clean
clean:
	@rm -rf $(BUILDDIR)/*

.PHONY: gh-pages
gh-pages:
	git checkout gh-pages && \
	cd .. && \
	git rm -rf . && git clean -ffxd && \
	git checkout master $(GH_PAGES_SOURCES) && \
	git reset HEAD && \
	cd docs && \
	rm -rf build && \
	make html && \
	cd .. && \
	mv -fv docs/build/html/* ./ && \
	touch .nojekyll && \
	rm -rf $(GH_PAGES_SOURCES) && \
	git add -A && \
	git commit -m ":pencil: Generated gh-pages for `git log master -1 --pretty=short --abbrev-commit`" && \
	git push origin gh-pages && \
	git checkout master

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@sphinx-apidoc -f -o "${BUILDDIR}/_modules" "../panda_autograsp/src"
	@sphinx-apidoc -f -o "${BUILDDIR}/_scripts" "../panda_autograsp/scripts"
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)