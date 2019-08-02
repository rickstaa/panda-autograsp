## Make file for the panda_autograsp sphinx documentation

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SOURCEDIR     = source
BUILDDIR      = _build
GH_PAGES_SOURCES = docs panda_autograsp tools

# Put it first so that "make" without argument is like "make help".
.PHONY: help Makefile
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: clean
clean:
	@rm -rf $(BUILDDIR)/*

.PHONY: gh-pages
gh-pages:
	@current_git_branch=$(git -C "$1" branch | sed  '/^\*/!d;s/\* //') && \
	git checkout gh-pages && \
	cd .. && \
	git rm -rf . && git clean -fxd && \
	git checkout $(current_git_branch) $(GH_PAGES_SOURCES) && \
	git reset HEAD && \
	cd docs && \
	sphinx-apidoc -o "${BUILDDIR}/_modules" ".."
	make html && \
	cd .. && \
	mv -fv docs/build/html/* ./ && \
	touch .nojekyll && \
	rm -rf $(GH_PAGES_SOURCES) && \
	git add -A && \
	git commit -m "Generated gh-pages for `git log master -1 --pretty=short --abbrev-commit`" && \
	git push origin --delete gh-pages && \
	git push origin gh-pages ; \
	git checkout $(current_git_branch)

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@sphinx-apidoc -o "${BUILDDIR}/_modules" ".."
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)