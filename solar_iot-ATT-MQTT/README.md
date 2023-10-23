# Solar_IoT_AG_CC

Sistema de telemetría IoT para el monitoreo de la instalación solar fotovoltaica del Centro Tecnológico FUNINTEC.

## Getting started

To make it easy for you to get started with GitLab, here's a list of recommended next steps.

Already a pro? Just edit this README.md and make it your own. Want to make it easy? [Use the template at the bottom](#editing-this-readme)!

## Add your files

- [ ] [Create](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/user/project/repository/web_editor.html#upload-a-file) files
- [ ] [Add files using the command line](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/gitlab-basics/add-file.html#add-a-file-using-the-command-line) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin https://gitlab.com/AL1T0/solar_iot.git
git branch -M main
git push -uf origin main
```

## Integrate with your tools

- [ ] [Set up project integrations](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://gitlab.com/AL1T0/solar_iot/-/settings/integrations)

## Collaborate with your team

- [ ] [Invite team members and collaborators](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/user/project/members/)
- [ ] [Create a new merge request](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/user/project/merge_requests/creating_merge_requests.html)
- [ ] [Automatically close issues from merge requests](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/user/project/issues/managing_issues.html#closing-issues-automatically)
- [ ] [Enable merge request approvals](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/user/project/merge_requests/approvals/)
- [ ] [Automatically merge when pipeline succeeds](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/user/project/merge_requests/merge_when_pipeline_succeeds.html)

## Test and Deploy

Use the built-in continuous integration in GitLab.

- [ ] [Get started with GitLab CI/CD](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/ci/quick_start/index.html)
- [ ] [Analyze your code for known vulnerabilities with Static Application Security Testing(SAST)](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/user/application_security/sast/)
- [ ] [Deploy to Kubernetes, Amazon EC2, or Amazon ECS using Auto Deploy](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/topics/autodevops/requirements.html)
- [ ] [Use pull-based deployments for improved Kubernetes management](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/user/clusters/agent/)
- [ ] [Set up protected environments](https://gitlab.com/-/experiment/new_project_readme_content:68d8d7cd626ab81a500abbee012ca5e6?https://docs.gitlab.com/ee/ci/environments/protected_environments.html)

***

## Description

- La plataforma de desarrollo, así como la placa física empleada son configuraciones del proyecto y se incluyen en el fichero `platformio.ini`.
- En dicho fichero pueden existir diferentes configuraciones para diferentes entornos, todos ellos comparten el mismo código fuente.
- El código fuente es independiente de la plataforma de desarrollo empleada y va localizado bajo la carpeta `src`.
- Basándose en el fichero `platformio.ini`, el sistema se encarga de descargar e instalar todos los requerimientos necesarios.
- La carpeta `lib` almacena las diferentes bibliotecas requeridas por la aplicación que se desarrolle.
- El usuario, a partir de una única herramienta y con una API sencilla, dispone de toda la funcionalidad que se requiere para crear, compilar y subir código.

Para compilar el proyecto, es necesario instalar la extensión de PlatformIO IDE y desde la página principal del IDE, levantar el proyecto y realizar la compilación utilizando el comando `PlatformIO:Build`.
