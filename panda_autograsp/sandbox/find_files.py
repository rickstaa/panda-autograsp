import os


def list_files(path='.', exclude=[], recursive=True):
    """Returns a list of files that are present in a folder.

    Parameters
    ----------
    path : str, optional
        Parent folder of which you want to list the files, by default '.' meaning
        the current working directory.
    exclude : list, optional
        A list of files you want to exclude, by default []
    recursive : bool, optional
        Option specifying whether you also want to list files of subfolders, by default True
    level : int, optional
        If recursive is enabled this specifies up till how many levels deep you want to list
        the files, by default 0 (Defined as all levels).

    Returns
    -------
    List
        A list containing the relative paths of all the files in the parent folder.
    """

    ## Get a list of files that are contained in the given path
    file_list = list()
    for dir_, _, files in os.walk(path):
        for file_name in files:
            rel_dir = os.path.relpath(dir_, path)
            rel_file = os.path.join(rel_dir, file_name)

            ## Add files to file list if they are not in exclude list ##
            if file_name not in exclude:
                file_list.append(rel_file)

        ## Break out of loop if recursive is disabled
        if not recursive:
            break
    return file_list

if __name__ == "__main__":
    list_files(path='../', recursive=False)