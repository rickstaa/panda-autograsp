.. _release_dev:

.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp

Release package
===================================

Before releasing the package, make sure the following steps are performed:

    #. Squash and merge your branch with the desired branch.
    #. :doc:`doc_dev` if needed.
    #. Bump the version using the `bumpversion tool <https://github.com/peritus/bumpversion>`_.
    #. Update the changelog using the `auto-changelog <https://github.com/CookPete/auto-changelog>`_ tool.
    #. Commit and push the changes to the remote.
    #. Create the right version tag and release using the github draft release tool.