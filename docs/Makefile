## Make file for the panda_autograsp sphinx documentation

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SPHINXPROJ    = panda_autograsp
SOURCEDIR     = source
BUILDDIR      = build
GH_PAGES_SOURCES = docs panda_autograsp

# Put it first so that "make" without argument is like "make help".
.PHONY: help Makefile
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

# Create make clean command
.PHONY: clean
clean:
	@rm -rf $(BUILDDIR)/*
	@echo "Build folder cleaned"

# Publish gh-pages command
.PHONY: gh-pages
gh-pages:
	cd .. && \
	git submodule update --recursive && \
	git checkout gh-pages && \
	git rm -rf . && git clean -fxd && \
	git checkout melodic-devel $(GH_PAGES_SOURCES) && \
	git reset HEAD && \
	cd docs && \
	make clean && \
	make html && \
	cd .. && \
	git add docs/build/html && \
	git clean -ffxd && \
	mv -fv docs/build/html/* ./ && \
	touch .nojekyll && \
	rm -rf $(GH_PAGES_SOURCES) && \
	git add -A && \
	git commit -m ":pencil: Generated gh-pages for `git log melodic-devel -1 --pretty=short --abbrev-commit`" && \
	git push origin gh-pages && \
	git checkout melodic-devel && \
	git submodule update --recursive

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	cd .. && \
	git submodule update --init --recursive
	@sphinx-apidoc -f -o "${SOURCEDIR}/_modules" "../panda_autograsp/src"
	@sphinx-apidoc -f -o "${SOURCEDIR}/_scripts" "../panda_autograsp/scripts"
	cat "${SOURCEDIR}/_scripts/modules.rst" | sed -e 's/scripts//g' | sed -e 's/=*//g' | sed -e 's/\.\. toctree:://g' | sed -e 's/:maxdepth: 4//g' | tail -n +6 >> "${SOURCEDIR}/_modules/modules.rst" 
	@sphinx-apidoc -f -o "${SOURCEDIR}/_nodes" "../panda_autograsp/nodes"
	cat "${SOURCEDIR}/_nodes/modules.rst" | sed -e 's/scripts//g' | sed -e 's/=*//g' | sed -e 's/\.\. toctree:://g' | sed -e 's/:maxdepth: 4//g' | tail -n +6 >> "${SOURCEDIR}/_modules/modules.rst" 
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
