package com.example.interfacegraphique;

import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.image.ImageView;
import javafx.scene.image.Image;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;


public class HelloController {

    @FXML
    private VBox vboxContainer;

    @FXML
    private ImageView backgroundImage;

    @FXML
    public void initialize() {
        Image image = new Image(getClass().getResource("/images/fondApp.jpg").toExternalForm());
        // Appliquer l'image à l'ImageView
        backgroundImage.setImage(image);
        backgroundImage.setPreserveRatio(false); // Conserve les proportions

        // Adapter la taille de l'image à la VBox
        backgroundImage.fitWidthProperty().bind(vboxContainer.widthProperty());
        backgroundImage.fitHeightProperty().bind(vboxContainer.heightProperty());

        // Permettre à l'image de s'étendre et de remplir l'espace
        VBox.setVgrow(backgroundImage, Priority.ALWAYS);}
}