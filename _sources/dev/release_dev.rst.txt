.. _release_dev:

.. _panda_autograsp: https://github.com/rickstaa/panda_autograsp

Release package
===================================

Before releasing the package, make sure the following steps are performed:

    #. Squash and merge your branch with the desired branch.
    #. Update the documentation according to :doc:`doc_dev` if needed.
    #. Bump the version using the `bumpversion tool <https://github.com/peritus/bumpversion>`_.
    #. Check the version of the current branch using the ``bumpversion --list`` command.
    #. Add a tag equal to the version specified in the last step (Check versioning guidelines below).
    #. Update the changelog using the `auto-changelog <https://github.com/CookPete/auto-changelog>`_ tool.
    #. Commit and push the changes to the remote.
    #. Create a release using the github draft release tool.

Versioning guidelines
----------------------

Please use the ROS version of the branch you are working on in the version tag
(Example: v1.0.1-kinetic for the kinetic branch and v1.0.1-melodic for the
melodic branch). Additionally please use the
`versioning guidelines specified at semver.org <https://semver.org/>`_.
