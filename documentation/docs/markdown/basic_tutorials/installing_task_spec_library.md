# Installing task specification libraries

TODO:

The mechanism is not yet implemented, but as an idea we should have a directory inside the application template where one copies (or creates a symbolic links) all libraries desired to be used within the application. Care must be used because different libraries can contain task specifications with the same filename, and if this happens the "is-" keyword from the JSON would failed. So perhaps also the name of the library should be added there (e.g. "is-<libalias>-<task_spec_name>").
The JSON Schema generation of the application template should then iterate over all the installed libraries.

An alias should be used for the libraries so that the names don't become huge (e.g. define an alias.text file where the alias is defined, or even better a JSON file where information of the libraries or meta-data is shown).

Information: alias, library description, author, licence, etc

Make this file required so that the schema generation fails if there is not such file.