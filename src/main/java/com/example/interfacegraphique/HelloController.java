package com.example.interfacegraphique;

import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.Button;
import javafx.scene.image.ImageView;
import javafx.scene.image.Image;
import javafx.scene.layout.Priority;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.VBox;
import javafx.application.Platform;


public class HelloController {

    @FXML
    private AnchorPane root;

    @FXML
    private ImageView backgroundImage;

    @FXML
    private Button loadingButton;

    @FXML
    public void initialize() {
        // Charger et afficher l'image de fond
        Image image = new Image(getClass().getResource("/images/fondApp.jpg").toExternalForm());
        backgroundImage.setImage(image);

        backgroundImage.setPreserveRatio(false); // Maintenir les proportions
        backgroundImage.setSmooth(true);
        backgroundImage.setCache(true);

        // Adapter l'image à la taille du root
        backgroundImage.fitWidthProperty().bind(root.widthProperty());
        backgroundImage.fitHeightProperty().bind(root.heightProperty());

        // Centrer le bouton dynamiquement quand la fenêtre change de taille
        root.widthProperty().addListener((obs, oldVal, newVal) -> updateButtonPosition());
        root.heightProperty().addListener((obs, oldVal, newVal) -> updateButtonPosition());

        // Positionner correctement le bouton au démarrage
        Platform.runLater(this::updateButtonPosition);
    }

    private void updateLayout() {
        adjustImageSize();
        updateButtonPosition();
    }

    private void adjustImageSize() {
        double paneRatio = root.getWidth() / root.getHeight();
        double imageRatio = backgroundImage.getImage().getWidth() / backgroundImage.getImage().getHeight();

        if (paneRatio > imageRatio) {
            backgroundImage.setFitWidth(root.getWidth());
            backgroundImage.setFitHeight(root.getWidth() / imageRatio);
        } else {
            backgroundImage.setFitHeight(root.getHeight());
            backgroundImage.setFitWidth(root.getHeight() * imageRatio);
        }
    }

    private void updateButtonPosition() {
        double centerX = backgroundImage.getLayoutX() + (backgroundImage.getFitWidth() - loadingButton.getWidth()) / 2;
        double centerY = backgroundImage.getLayoutY() + (backgroundImage.getFitHeight() - loadingButton.getHeight()) / 2;

        loadingButton.setLayoutX(centerX);
        loadingButton.setLayoutY(centerY);
    }


}
